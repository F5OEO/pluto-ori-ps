/*
 * obs_stream_control.c
 *
 * Contrôle du stream ET de l'enregistrement OBS via obs-websocket v5 en C.
 * Profil : Avancé, enregistrement en sortie personnalisée (Custom FFmpeg).
 *
 * Stop → change params → restart (stream, recording, ou les deux).
 *
 * Dépendances : libwebsockets, openssl, json-c
 *
 * Compilation :
 *   gcc -o obs_stream_control obs_stream_control.c \
 *       -lwebsockets -lssl -lcrypto -ljson-c
 *
 * Usage :
 *   ./obs_stream_control -m stream  -b 6000
 *   ./obs_stream_control -m record  -b 4500 -e hevc_nvenc -f mpegts -u "srt://server:9000"
 *   ./obs_stream_control -m both    -b 6000 --rec-bitrate 8000
 *   ./obs_stream_control -m record  -b 3000 -g 50 --ff-custom "profile=main level=4.0"
 *
 * Paramètres modifiables (mode Avancé / Custom FFmpeg) :
 *
 *   basic.ini [AdvOut]         websocket category/name        option CLI
 *   ─────────────────────────────────────────────────────────────────────
 *   FFVBitrate                 AdvOut / FFVBitrate             --rec-bitrate
 *   FFABitrate                 AdvOut / FFABitrate             --rec-abitrate
 *   FFVEncoder                 AdvOut / FFVEncoder             -e / --encoder
 *   FFAEncoder                 AdvOut / FFAEncoder             --aencoder
 *   FFFormat                   AdvOut / FFFormat               -f / --format
 *   FFVCustom                  AdvOut / FFVCustom              --ff-custom
 *   FFACustom                  AdvOut / FFACustom              --ff-acustom
 *   FFMCustom                  AdvOut / FFMCustom              --ff-mcustom
 *   FFURL                      AdvOut / FFURL                  -u / --url
 *   FFVGOPSize                 AdvOut / FFVGOPSize             -g / --gop
 *   FFOutputToFile             AdvOut / FFOutputToFile         --to-file
 *   FFRescaleRes               AdvOut / FFRescaleRes           --rescale
 *
 *   Stream (mode Avancé) :
 *   Encoder bitrate            AdvOut / Encoder (+ settings)
 *   VBitrate simplifié via     SimpleOutput / VBitrate         -b / --bitrate
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <libwebsockets.h>
#include <openssl/sha.h>
#include <openssl/bio.h>
#include <openssl/evp.h>
#include <openssl/buffer.h>
#include <json-c/json.h>

/* ── Mode de contrôle ───────────────────────────────────────────── */

typedef enum {
    MODE_STREAM = 1,
    MODE_RECORD = 2,
    MODE_BOTH   = 3
} control_mode_t;

/* ── Paramètre à modifier ──────────────────────────────────────── */

typedef struct param_change {
    const char          *category;  /* ex: "AdvOut", "SimpleOutput" */
    const char          *name;      /* ex: "FFVBitrate" */
    const char          *value;
    struct param_change *next;
} param_change_t;

/* ── Configuration ──────────────────────────────────────────────── */

typedef struct {
    const char     *host;
    int             port;
    const char     *password;
    control_mode_t  mode;

    /* Listes chaînées de paramètres à modifier */
    param_change_t *stream_params;  /* params stream */
    param_change_t *rec_params;     /* params enregistrement */
} obs_config_t;

/* ── Machine à états ────────────────────────────────────────────── */

typedef enum {
    STATE_WAIT_HELLO,
    STATE_WAIT_IDENTIFIED,

    STATE_STOP_STREAM,
    STATE_WAIT_STOP_STREAM,

    STATE_STOP_RECORD,
    STATE_WAIT_STOP_RECORD,

    STATE_APPLY_STREAM_PARAMS,
    STATE_WAIT_STREAM_PARAM,

    STATE_APPLY_REC_PARAMS,
    STATE_WAIT_REC_PARAM,

    STATE_START_STREAM,
    STATE_WAIT_START_STREAM,

    STATE_START_RECORD,
    STATE_WAIT_START_RECORD,

    STATE_DONE
} client_state_t;

typedef struct {
    obs_config_t    config;
    client_state_t  state;
    struct lws     *wsi;
    int             request_id;
    param_change_t *current_param;  /* param en cours d'application */
} session_t;

static session_t g_session;
static int       g_force_exit = 0;

/* ── Gestion des paramètres ─────────────────────────────────────── */

static param_change_t *param_new(const char *cat, const char *name, const char *val)
{
    param_change_t *p = calloc(1, sizeof(*p));
    p->category = cat;
    p->name     = name;
    p->value    = val;
    return p;
}

static void param_append(param_change_t **head, param_change_t *p)
{
    if (!*head) { *head = p; return; }
    param_change_t *tail = *head;
    while (tail->next) tail = tail->next;
    tail->next = p;
}

static int param_count(param_change_t *head)
{
    int n = 0;
    for (; head; head = head->next) n++;
    return n;
}

/* ── Utilitaires base64 / SHA256 ────────────────────────────────── */

static char *base64_encode(const unsigned char *data, int len)
{
    BIO *bio, *b64;
    BUF_MEM *bptr;

    b64  = BIO_new(BIO_f_base64());
    bio  = BIO_new(BIO_s_mem());
    bio  = BIO_push(b64, bio);

    BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);
    BIO_write(bio, data, len);
    BIO_flush(bio);
    BIO_get_mem_ptr(bio, &bptr);

    char *result = malloc(bptr->length + 1);
    memcpy(result, bptr->data, bptr->length);
    result[bptr->length] = '\0';

    BIO_free_all(bio);
    return result;
}

static void sha256_hash(const unsigned char *data, size_t len, unsigned char *out)
{
    SHA256_CTX ctx;
    SHA256_Init(&ctx);
    SHA256_Update(&ctx, data, len);
    SHA256_Final(out, &ctx);
}

static char *compute_auth(const char *password, const char *challenge, const char *salt)
{
    unsigned char hash[SHA256_DIGEST_LENGTH];
    char *concat;
    size_t len;

    len = strlen(password) + strlen(salt);
    concat = malloc(len + 1);
    sprintf(concat, "%s%s", password, salt);
    sha256_hash((unsigned char *)concat, len, hash);
    free(concat);
    char *secret = base64_encode(hash, SHA256_DIGEST_LENGTH);

    len = strlen(secret) + strlen(challenge);
    concat = malloc(len + 1);
    sprintf(concat, "%s%s", secret, challenge);
    sha256_hash((unsigned char *)concat, len, hash);
    free(concat);
    free(secret);

    return base64_encode(hash, SHA256_DIGEST_LENGTH);
}

/* ── Envoi WebSocket ────────────────────────────────────────────── */

static int ws_send(struct lws *wsi, const char *msg)
{
    size_t len = strlen(msg);
    unsigned char *buf = malloc(LWS_PRE + len);
    if (!buf) return -1;

    memcpy(buf + LWS_PRE, msg, len);
    int ret = lws_write(wsi, buf + LWS_PRE, len, LWS_WRITE_TEXT);
    free(buf);

    printf("  → %s\n", msg);
    return ret;
}

/* ── Construction JSON ──────────────────────────────────────────── */

static char *build_identify(const char *auth_token)
{
    struct json_object *root = json_object_new_object();
    struct json_object *d    = json_object_new_object();

    json_object_object_add(root, "op", json_object_new_int(1));
    json_object_object_add(d, "rpcVersion", json_object_new_int(1));
    if (auth_token)
        json_object_object_add(d, "authentication", json_object_new_string(auth_token));
    json_object_object_add(root, "d", d);

    const char *str = json_object_to_json_string(root);
    char *result = strdup(str);
    json_object_put(root);
    return result;
}

static char *build_request(const char *req_type, int req_id, struct json_object *data)
{
    struct json_object *root = json_object_new_object();
    struct json_object *d    = json_object_new_object();
    char id_str[16];
    snprintf(id_str, sizeof(id_str), "%d", req_id);

    json_object_object_add(root, "op", json_object_new_int(6));
    json_object_object_add(d, "requestType", json_object_new_string(req_type));
    json_object_object_add(d, "requestId",   json_object_new_string(id_str));
    if (data)
        json_object_object_add(d, "requestData", data);
    json_object_object_add(root, "d", d);

    const char *str = json_object_to_json_string(root);
    char *result = strdup(str);
    json_object_put(root);
    return result;
}

static char *build_set_profile_param(int req_id, const param_change_t *p)
{
    struct json_object *data = json_object_new_object();
    json_object_object_add(data, "parameterCategory",
                           json_object_new_string(p->category));
    json_object_object_add(data, "parameterName",
                           json_object_new_string(p->name));
    json_object_object_add(data, "parameterValue",
                           json_object_new_string(p->value));
    return build_request("SetProfileParameter", req_id, data);
}

/* ── Transitions d'état ─────────────────────────────────────────── */

static client_state_t first_start_state(session_t *s);

static client_state_t next_after_stop_stream(session_t *s)
{
    if (s->config.mode & MODE_RECORD)
        return STATE_STOP_RECORD;
    if (s->config.stream_params)
        return STATE_APPLY_STREAM_PARAMS;
    return first_start_state(s);
}

static client_state_t first_param_state(session_t *s)
{
    if ((s->config.mode & MODE_STREAM) && s->config.stream_params)
        return STATE_APPLY_STREAM_PARAMS;
    if ((s->config.mode & MODE_RECORD) && s->config.rec_params)
        return STATE_APPLY_REC_PARAMS;
    return first_start_state(s);
}

static client_state_t first_start_state(session_t *s)
{
    if (s->config.mode & MODE_STREAM)
        return STATE_START_STREAM;
    return STATE_START_RECORD;
}

static client_state_t next_after_stream_params(session_t *s)
{
    if ((s->config.mode & MODE_RECORD) && s->config.rec_params)
        return STATE_APPLY_REC_PARAMS;
    return first_start_state(s);
}

static client_state_t next_after_rec_params(session_t *s)
{
    return first_start_state(s);
}

static client_state_t next_after_start_stream(session_t *s)
{
    if (s->config.mode & MODE_RECORD)
        return STATE_START_RECORD;
    return STATE_DONE;
}

/* ── Traitement des messages ────────────────────────────────────── */

static void handle_message(session_t *sess, const char *msg)
{
    struct json_object *root = json_tokener_parse(msg);
    if (!root) { fprintf(stderr, "  ✗ JSON invalide\n"); return; }

    struct json_object *op_obj;
    json_object_object_get_ex(root, "op", &op_obj);
    int op = json_object_get_int(op_obj);

    /* Vérifier erreurs dans les réponses */
    if (op == 7) {
        struct json_object *d, *status, *result_obj;
        json_object_object_get_ex(root, "d", &d);
        if (d && json_object_object_get_ex(d, "requestStatus", &status)) {
            json_object_object_get_ex(status, "result", &result_obj);
            if (!json_object_get_boolean(result_obj)) {
                struct json_object *code_obj, *comment_obj;
                json_object_object_get_ex(status, "code", &code_obj);
                json_object_object_get_ex(status, "comment", &comment_obj);
                printf("  ⚠ Erreur: code=%d %s\n",
                    json_object_get_int(code_obj),
                    comment_obj ? json_object_get_string(comment_obj) : "");
            }
        }
    }

    switch (sess->state) {

    /* ── Auth ──────────────────────────────────────────────────── */

    case STATE_WAIT_HELLO: {
        if (op != 0) break;
        char *auth_token = NULL;
        struct json_object *d, *auth_obj;
        json_object_object_get_ex(root, "d", &d);

        if (json_object_object_get_ex(d, "authentication", &auth_obj)) {
            struct json_object *chal, *salt_obj;
            json_object_object_get_ex(auth_obj, "challenge", &chal);
            json_object_object_get_ex(auth_obj, "salt", &salt_obj);
            auth_token = compute_auth(
                sess->config.password,
                json_object_get_string(chal),
                json_object_get_string(salt_obj)
            );
            printf("  ✓ Auth calculée\n");
        }

        char *identify = build_identify(auth_token);
        ws_send(sess->wsi, identify);
        free(identify);
        free(auth_token);
        sess->state = STATE_WAIT_IDENTIFIED;
        break;
    }

    case STATE_WAIT_IDENTIFIED:
        if (op != 2) break;
        printf("  ✓ Identifié\n\n");
        if (sess->config.mode & MODE_STREAM)
            sess->state = STATE_STOP_STREAM;
        else
            sess->state = STATE_STOP_RECORD;
        lws_callback_on_writable(sess->wsi);
        break;

    /* ── Arrêts ────────────────────────────────────────────────── */

    case STATE_WAIT_STOP_STREAM:
        if (op != 7) break;
        printf("  ✓ Stream stoppé\n");
        sess->state = next_after_stop_stream(sess);
        if (sess->state != STATE_STOP_RECORD) {
            printf("  … Attente 2s…\n");
            usleep(2000000);
        }
        lws_callback_on_writable(sess->wsi);
        break;

    case STATE_WAIT_STOP_RECORD:
        if (op != 7) break;
        printf("  ✓ Enregistrement stoppé\n");
        printf("  … Attente 2s…\n");
        usleep(2000000);
        sess->state = first_param_state(sess);
        lws_callback_on_writable(sess->wsi);
        break;

    /* ── Application des paramètres (un par un) ────────────────── */

    case STATE_WAIT_STREAM_PARAM:
        if (op != 7) break;
        printf("  ✓ Stream param [%s/%s] = %s\n",
               sess->current_param->category,
               sess->current_param->name,
               sess->current_param->value);
        sess->current_param = sess->current_param->next;
        if (sess->current_param) {
            sess->state = STATE_APPLY_STREAM_PARAMS;
        } else {
            sess->state = next_after_stream_params(sess);
        }
        lws_callback_on_writable(sess->wsi);
        break;

    case STATE_WAIT_REC_PARAM:
        if (op != 7) break;
        printf("  ✓ Rec param [%s/%s] = %s\n",
               sess->current_param->category,
               sess->current_param->name,
               sess->current_param->value);
        sess->current_param = sess->current_param->next;
        if (sess->current_param) {
            sess->state = STATE_APPLY_REC_PARAMS;
        } else {
            sess->state = next_after_rec_params(sess);
        }
        lws_callback_on_writable(sess->wsi);
        break;

    /* ── Démarrages ────────────────────────────────────────────── */

    case STATE_WAIT_START_STREAM:
        if (op != 7) break;
        printf("  ✓ Stream relancé\n");
        sess->state = next_after_start_stream(sess);
        if (sess->state == STATE_DONE)
            g_force_exit = 1;
        else
            lws_callback_on_writable(sess->wsi);
        break;

    case STATE_WAIT_START_RECORD:
        if (op != 7) break;
        printf("  ✓ Enregistrement relancé\n");
        sess->state = STATE_DONE;
        g_force_exit = 1;
        break;

    default:
        break;
    }

    json_object_put(root);
}

/* ── Callback libwebsockets ─────────────────────────────────────── */

static int callback_obs(struct lws *wsi, enum lws_callback_reasons reason,
                        void *user, void *in, size_t len)
{
    (void)user; (void)len;
    session_t *sess = &g_session;

    switch (reason) {

    case LWS_CALLBACK_CLIENT_ESTABLISHED:
        printf("✓ Connexion WebSocket établie\n");
        sess->wsi   = wsi;
        sess->state = STATE_WAIT_HELLO;
        break;

    case LWS_CALLBACK_CLIENT_RECEIVE:
        handle_message(sess, (const char *)in);
        break;

    case LWS_CALLBACK_CLIENT_WRITEABLE: {
        char *msg = NULL;

        switch (sess->state) {

        case STATE_STOP_STREAM:
            printf("[étape] Arrêt du stream…\n");
            msg = build_request("StopStream", ++sess->request_id, NULL);
            sess->state = STATE_WAIT_STOP_STREAM;
            break;

        case STATE_STOP_RECORD:
            printf("[étape] Arrêt de l'enregistrement…\n");
            msg = build_request("StopRecord", ++sess->request_id, NULL);
            sess->state = STATE_WAIT_STOP_RECORD;
            break;

        case STATE_APPLY_STREAM_PARAMS:
            if (!sess->current_param)
                sess->current_param = sess->config.stream_params;
            printf("[étape] Set stream %s/%s = %s\n",
                   sess->current_param->category,
                   sess->current_param->name,
                   sess->current_param->value);
            msg = build_set_profile_param(++sess->request_id, sess->current_param);
            sess->state = STATE_WAIT_STREAM_PARAM;
            break;

        case STATE_APPLY_REC_PARAMS:
            if (!sess->current_param)
                sess->current_param = sess->config.rec_params;
            printf("[étape] Set rec %s/%s = %s\n",
                   sess->current_param->category,
                   sess->current_param->name,
                   sess->current_param->value);
            msg = build_set_profile_param(++sess->request_id, sess->current_param);
            sess->state = STATE_WAIT_REC_PARAM;
            break;

        case STATE_START_STREAM:
            printf("[étape] Relance du stream…\n");
            msg = build_request("StartStream", ++sess->request_id, NULL);
            sess->state = STATE_WAIT_START_STREAM;
            break;

        case STATE_START_RECORD:
            printf("[étape] Relance de l'enregistrement…\n");
            msg = build_request("StartRecord", ++sess->request_id, NULL);
            sess->state = STATE_WAIT_START_RECORD;
            break;

        default:
            break;
        }

        if (msg) {
            ws_send(wsi, msg);
            free(msg);
        }
        break;
    }

    case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
        fprintf(stderr, "✗ Erreur: %s\n", in ? (const char *)in : "inconnue");
        g_force_exit = 1;
        break;

    case LWS_CALLBACK_CLOSED:
        printf("Connexion fermée\n");
        g_force_exit = 1;
        break;

    default:
        break;
    }

    return 0;
}

static const struct lws_protocols protocols[] = {
    { "obs-websocket", callback_obs, 0, 65536 },
    { NULL, NULL, 0, 0 }
};

/* ── Usage ──────────────────────────────────────────────────────── */

static void usage(const char *prog)
{
    printf(
    "Usage: %s [options]\n\n"
    "Connexion:\n"
    "  -H <host>          Hôte OBS (défaut: localhost)\n"
    "  -p <port>          Port WebSocket (défaut: 4455)\n"
    "  -w <password>      Mot de passe\n\n"
    "Mode:\n"
    "  -m <mode>          stream | record | both (défaut: stream)\n\n"
    "Paramètres stream (SimpleOutput):\n"
    "  -b <bitrate>       Bitrate vidéo stream (kbps)\n\n"
    "Paramètres enregistrement (AdvOut / Custom FFmpeg):\n"
    "  --rec-bitrate <n>  FFVBitrate  - bitrate vidéo rec (kbps)\n"
    "  --rec-abitrate <n> FFABitrate  - bitrate audio rec (kbps)\n"
    "  -e <encoder>       FFVEncoder  - encodeur vidéo (libx264, hevc_nvenc…)\n"
    "  --aencoder <enc>   FFAEncoder  - encodeur audio (aac, opus…)\n"
    "  -f <format>        FFFormat    - conteneur (mpegts, mp4, mkv…)\n"
    "  -u <url>           FFURL       - URL de sortie (srt://, udp://…)\n"
    "  -g <gop>           FFVGOPSize  - taille du GOP\n"
    "  --ff-custom <str>  FFVCustom   - params encodeur vidéo FFmpeg\n"
    "  --ff-acustom <str> FFACustom   - params encodeur audio FFmpeg\n"
    "  --ff-mcustom <str> FFMCustom   - params muxer FFmpeg\n"
    "  --rescale <WxH>    FFRescaleRes - résolution de rescale\n"
    "  --to-file <bool>   FFOutputToFile - true/false\n\n"
    "Exemples:\n"
    "  %s -m stream -b 6000\n"
    "  %s -m record --rec-bitrate 4500 -e hevc_nvenc -f mpegts\n"
    "  %s -m record --rec-bitrate 3000 -u \"srt://server:9000\" -g 50\n"
    "  %s -m both -b 6000 --rec-bitrate 8000 --ff-custom \"profile=high preset=slow\"\n",
    prog, prog, prog, prog, prog);
}

/* ── Main ───────────────────────────────────────────────────────── */

int main(int argc, char **argv)
{
    memset(&g_session, 0, sizeof(g_session));
    g_session.config.host     = "localhost";
    g_session.config.port     = 4455;
    g_session.config.password = "";
    g_session.config.mode     = MODE_STREAM;

    static struct option long_opts[] = {
        {"rec-bitrate",  required_argument, 0, 'R'},
        {"rec-abitrate", required_argument, 0, 'A'},
        {"aencoder",     required_argument, 0, 'E'},
        {"ff-custom",    required_argument, 0, 'C'},
        {"ff-acustom",   required_argument, 0, 'D'},
        {"ff-mcustom",   required_argument, 0, 'M'},
        {"rescale",      required_argument, 0, 'S'},
        {"to-file",      required_argument, 0, 'T'},
        {"help",         no_argument,       0, '?'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "b:m:H:p:w:e:f:u:g:?", long_opts, NULL)) != -1) {
        switch (opt) {

        /* ── Connexion ─────────────────────────────────────────── */
        case 'H': g_session.config.host     = optarg; break;
        case 'p': g_session.config.port     = atoi(optarg); break;
        case 'w': g_session.config.password = optarg; break;

        /* ── Mode ──────────────────────────────────────────────── */
        case 'm':
            if      (!strcmp(optarg, "stream")) g_session.config.mode = MODE_STREAM;
            else if (!strcmp(optarg, "record")) g_session.config.mode = MODE_RECORD;
            else if (!strcmp(optarg, "both"))   g_session.config.mode = MODE_BOTH;
            else { fprintf(stderr, "Mode invalide: %s\n", optarg); return 1; }
            break;

        /* ── Stream params ─────────────────────────────────────── */
        case 'b':
            param_append(&g_session.config.stream_params,
                         param_new("SimpleOutput", "VBitrate", optarg));
            break;

        /* ── Recording params (AdvOut / Custom FFmpeg) ─────────── */
        case 'R':
            param_append(&g_session.config.rec_params,
                         param_new("AdvOut", "FFVBitrate", optarg));
            break;
        case 'A':
            param_append(&g_session.config.rec_params,
                         param_new("AdvOut", "FFABitrate", optarg));
            break;
        case 'e':
            param_append(&g_session.config.rec_params,
                         param_new("AdvOut", "FFVEncoder", optarg));
            break;
        case 'E':
            param_append(&g_session.config.rec_params,
                         param_new("AdvOut", "FFAEncoder", optarg));
            break;
        case 'f':
            param_append(&g_session.config.rec_params,
                         param_new("AdvOut", "FFFormat", optarg));
            break;
        case 'u':
            param_append(&g_session.config.rec_params,
                         param_new("AdvOut", "FFURL", optarg));
            break;
        case 'g':
            param_append(&g_session.config.rec_params,
                         param_new("AdvOut", "FFVGOPSize", optarg));
            break;
        case 'C':
            param_append(&g_session.config.rec_params,
                         param_new("AdvOut", "FFVCustom", optarg));
            break;
        case 'D':
            param_append(&g_session.config.rec_params,
                         param_new("AdvOut", "FFACustom", optarg));
            break;
        case 'M':
            param_append(&g_session.config.rec_params,
                         param_new("AdvOut", "FFMCustom", optarg));
            break;
        case 'S':
            param_append(&g_session.config.rec_params,
                         param_new("AdvOut", "FFRescaleRes", optarg));
            break;
        case 'T':
            param_append(&g_session.config.rec_params,
                         param_new("AdvOut", "FFOutputToFile", optarg));
            break;

        case '?':
        default:
            usage(argv[0]);
            return 0;
        }
    }

    const char *mode_str =
        g_session.config.mode == MODE_BOTH   ? "stream + record" :
        g_session.config.mode == MODE_RECORD ? "record" : "stream";

    printf("═══════════════════════════════════════════════════\n");
    printf("  OBS Stream/Record Control (Adv/Custom FFmpeg)\n");
    printf("  Host:         %s:%d\n", g_session.config.host, g_session.config.port);
    printf("  Mode:         %s\n", mode_str);
    printf("  Stream params: %d\n", param_count(g_session.config.stream_params));
    printf("  Rec params:    %d\n", param_count(g_session.config.rec_params));
    printf("═══════════════════════════════════════════════════\n\n");

    /* Lister les paramètres qui seront appliqués */
    for (param_change_t *p = g_session.config.stream_params; p; p = p->next)
        printf("  stream: %s/%s = %s\n", p->category, p->name, p->value);
    for (param_change_t *p = g_session.config.rec_params; p; p = p->next)
        printf("  rec:    %s/%s = %s\n", p->category, p->name, p->value);
    if (g_session.config.stream_params || g_session.config.rec_params)
        printf("\n");

    struct lws_context_creation_info ctx_info;
    memset(&ctx_info, 0, sizeof(ctx_info));
    ctx_info.port      = CONTEXT_PORT_NO_LISTEN;
    ctx_info.protocols = protocols;
    ctx_info.options   = LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;

    struct lws_context *context = lws_create_context(&ctx_info);
    if (!context) {
        fprintf(stderr, "✗ Impossible de créer le contexte LWS\n");
        return 1;
    }

    struct lws_client_connect_info conn_info;
    memset(&conn_info, 0, sizeof(conn_info));
    conn_info.context  = context;
    conn_info.address  = g_session.config.host;
    conn_info.port     = g_session.config.port;
    conn_info.path     = "/";
    conn_info.host     = g_session.config.host;
    conn_info.origin   = g_session.config.host;
    conn_info.protocol = protocols[0].name;

    struct lws *wsi = lws_client_connect_via_info(&conn_info);
    if (!wsi) {
        fprintf(stderr, "✗ Impossible de se connecter\n");
        lws_context_destroy(context);
        return 1;
    }

    while (!g_force_exit)
        lws_service(context, 100);

    lws_context_destroy(context);
    printf("\n✓ Terminé.\n");
    return 0;
}
