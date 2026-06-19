/*
 * obs_stream_control.cpp
 *
 * Contrôle du stream ET de l'enregistrement OBS via obs-websocket v5.
 * Profil : Avancé, enregistrement en sortie personnalisée (Custom FFmpeg).
 *
 * Stop → change params → restart (stream, recording, ou les deux).
 *
 * Dépendances : libwebsockets, openssl, nlohmann/json (header-only)
 *
 * Compilation :
 *   g++ -o obs_stream_control obs_stream_control.cpp \
 *       -lwebsockets -lssl -lcrypto
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
 *   FFRescale                  AdvOut / FFRescale              (auto avec --rescale)
 *   FFRescaleRes               AdvOut / FFRescaleRes           --rescale
 *   FFSamplingRate             AdvOut / FFSamplingRate         --sample-rate
 *   FFIgnoreCompat             AdvOut / FFIgnoreCompat         --ignore-compat
 *   FFAudioMixes               AdvOut / FFAudioMixes           --audio-mixes
 *
 *   basic.ini [Video]          websocket category/name        option CLI
 *   ─────────────────────────────────────────────────────────────────────
 *   FPSType                    Video / FPSType                 (auto avec --fps)
 *   FPSInt                     Video / FPSInt                  --fps
 *
 *   Stream (mode Avancé) :
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
#include "json.hpp"

using json = nlohmann::json;

/* ── Mode de contrôle ───────────────────────────────────────────── */

typedef enum {
    MODE_STREAM = 1,
    MODE_RECORD = 2,
    MODE_BOTH   = 3
} control_mode_t;

/* ── Paramètre à modifier ──────────────────────────────────────── */

typedef struct param_change {
    const char          *category;
    const char          *name;
    const char          *value;
    struct param_change *next;
} param_change_t;

/* ── Configuration ──────────────────────────────────────────────── */

typedef struct {
    const char     *host;
    int             port;
    const char     *password;
    control_mode_t  mode;
    param_change_t *stream_params;
    param_change_t *rec_params;
    int             fps_int;   /* 0 = pas de changement FPS */
} obs_config_t;

/* ── Machine à états ────────────────────────────────────────────── */

typedef enum {
    STATE_WAIT_HELLO,
    STATE_WAIT_IDENTIFIED,
    STATE_STOP_STREAM,
    STATE_WAIT_STOP_STREAM,
    STATE_STOP_RECORD,
    STATE_WAIT_STOP_RECORD,
    STATE_GET_VIDEO_SETTINGS,
    STATE_WAIT_GET_VIDEO_SETTINGS,
    STATE_APPLY_VIDEO_SETTINGS,
    STATE_WAIT_VIDEO_SETTINGS,
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
    param_change_t *current_param;
    /* settings vidéo courants récupérés via GetVideoSettings */
    int             base_width, base_height;
    int             output_width, output_height;
    int             fps_num, fps_den;
} session_t;

static session_t g_session;
static int       g_force_exit = 0;

/* ── Gestion des paramètres ─────────────────────────────────────── */

static param_change_t *param_new(const char *cat, const char *name, const char *val)
{
    param_change_t *p = (param_change_t *)calloc(1, sizeof(*p));
    p->category = cat; p->name = name; p->value = val;
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
    b64 = BIO_new(BIO_f_base64());
    bio = BIO_new(BIO_s_mem());
    bio = BIO_push(b64, bio);
    BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);
    BIO_write(bio, data, len);
    BIO_flush(bio);
    BIO_get_mem_ptr(bio, &bptr);
    char *result = (char *)malloc(bptr->length + 1);
    memcpy(result, bptr->data, bptr->length);
    result[bptr->length] = '\0';
    BIO_free_all(bio);
    return result;
}

static void sha256_hash(const unsigned char *data, size_t len, unsigned char *out)
{
    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    unsigned int outlen = 0;
    EVP_DigestInit_ex(ctx, EVP_sha256(), NULL);
    EVP_DigestUpdate(ctx, data, len);
    EVP_DigestFinal_ex(ctx, out, &outlen);
    EVP_MD_CTX_free(ctx);
}

static char *compute_auth(const char *password, const char *challenge, const char *salt)
{
    unsigned char hash[32];
    size_t len = strlen(password) + strlen(salt);
    char *concat = (char *)malloc(len + 1);
    sprintf(concat, "%s%s", password, salt);
    sha256_hash((unsigned char *)concat, len, hash);
    free(concat);
    char *secret = base64_encode(hash, 32);

    len = strlen(secret) + strlen(challenge);
    concat = (char *)malloc(len + 1);
    sprintf(concat, "%s%s", secret, challenge);
    sha256_hash((unsigned char *)concat, len, hash);
    free(concat);
    free(secret);
    return base64_encode(hash, 32);
}

/* ── Envoi WebSocket ────────────────────────────────────────────── */

static int ws_send(struct lws *wsi, const char *msg)
{
    size_t len = strlen(msg);
    unsigned char *buf = (unsigned char *)malloc(LWS_PRE + len);
    if (!buf) return -1;
    memcpy(buf + LWS_PRE, msg, len);
    int ret = lws_write(wsi, buf + LWS_PRE, len, LWS_WRITE_TEXT);
    free(buf);
    printf("  → %s\n", msg);
    return ret;
}

/* ── Construction JSON (nlohmann) ───────────────────────────────── */

static char *build_identify(const char *auth_token)
{
    json root;
    root["op"] = 1;
    root["d"]["rpcVersion"] = 1;
    if (auth_token && *auth_token)
        root["d"]["authentication"] = auth_token;
    return strdup(root.dump().c_str());
}

static char *build_request(const char *req_type, int req_id, const json *data)
{
    char id_str[16];
    snprintf(id_str, sizeof(id_str), "%d", req_id);
    json root;
    root["op"] = 6;
    root["d"]["requestType"] = req_type;
    root["d"]["requestId"]   = id_str;
    if (data)
        root["d"]["requestData"] = *data;
    return strdup(root.dump().c_str());
}

static char *build_set_profile_param(int req_id, const param_change_t *p)
{
    json data;
    data["parameterCategory"] = p->category;
    data["parameterName"]     = p->name;
    data["parameterValue"]    = p->value;
    return build_request("SetProfileParameter", req_id, &data);
}

static char *build_get_video_settings(int req_id)
{
    return build_request("GetVideoSettings", req_id, nullptr);
}

static char *build_set_video_settings(int req_id, const session_t *s)
{
    json data;
    data["fpsNumerator"]   = s->config.fps_int;
    data["fpsDenominator"] = 1;
    data["baseWidth"]      = s->base_width;
    data["baseHeight"]     = s->base_height;
    data["outputWidth"]    = s->output_width;
    data["outputHeight"]   = s->output_height;
    return build_request("SetVideoSettings", req_id, &data);
}

/* ── Transitions d'état ─────────────────────────────────────────── */

static client_state_t first_start_state(session_t *s);
static client_state_t first_param_state(session_t *s);

static client_state_t next_after_stop_stream(session_t *s)
{
    if (s->config.mode & MODE_RECORD) return STATE_STOP_RECORD;
    return first_param_state(s);
}

static client_state_t first_param_state(session_t *s)
{
    if (s->config.fps_int > 0)                                     return STATE_GET_VIDEO_SETTINGS;
    if ((s->config.mode & MODE_STREAM) && s->config.stream_params) return STATE_APPLY_STREAM_PARAMS;
    if ((s->config.mode & MODE_RECORD) && s->config.rec_params)    return STATE_APPLY_REC_PARAMS;
    return first_start_state(s);
}

static client_state_t next_after_video_settings(session_t *s)
{
    if ((s->config.mode & MODE_STREAM) && s->config.stream_params) return STATE_APPLY_STREAM_PARAMS;
    if ((s->config.mode & MODE_RECORD) && s->config.rec_params)    return STATE_APPLY_REC_PARAMS;
    return first_start_state(s);
}

static client_state_t first_start_state(session_t *s)
{
    if (s->config.mode & MODE_STREAM) return STATE_START_STREAM;
    return STATE_START_RECORD;
}

static client_state_t next_after_stream_params(session_t *s)
{
    if ((s->config.mode & MODE_RECORD) && s->config.rec_params) return STATE_APPLY_REC_PARAMS;
    return first_start_state(s);
}

static client_state_t next_after_rec_params(session_t *s)   { return first_start_state(s); }
static client_state_t next_after_start_stream(session_t *s) { return (s->config.mode & MODE_RECORD) ? STATE_START_RECORD : STATE_DONE; }

/* ── Traitement des messages ────────────────────────────────────── */

static void handle_message(session_t *sess, const char *msg)
{
    json root;
    try { root = json::parse(msg); }
    catch (...) { fprintf(stderr, "  ✗ JSON invalide\n"); return; }

    int op = root.value("op", -1);

    if (op == 7 && root.contains("d")) {
        auto &d = root["d"];
        if (d.contains("requestStatus")) {
            auto &st = d["requestStatus"];
            if (!st.value("result", true)) {
                printf("  ⚠ Erreur: code=%d %s\n",
                    st.value("code", 0),
                    st.value("comment", std::string("")).c_str());
            }
        }
    }

    switch (sess->state) {

    case STATE_WAIT_HELLO: {
        if (op != 0) break;
        char *auth_token = nullptr;
        if (root.contains("d") && root["d"].contains("authentication")) {
            auto &auth = root["d"]["authentication"];
            std::string chal = auth.value("challenge", "");
            std::string salt = auth.value("salt", "");
            auth_token = compute_auth(sess->config.password, chal.c_str(), salt.c_str());
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
        sess->state = (sess->config.mode & MODE_STREAM) ? STATE_STOP_STREAM : STATE_STOP_RECORD;
        lws_callback_on_writable(sess->wsi);
        break;

    case STATE_WAIT_STOP_STREAM:
        if (op != 7) break;
        printf("  ✓ Stream stoppé\n");
        sess->state = next_after_stop_stream(sess);
        if (sess->state != STATE_STOP_RECORD) { printf("  … Attente 2s…\n"); usleep(2000000); }
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

    case STATE_WAIT_GET_VIDEO_SETTINGS:
        if (op != 7) break;
        if (root.contains("d") && root["d"].contains("responseData")) {
            auto &rd = root["d"]["responseData"];
            sess->base_width    = rd.value("baseWidth",    1920);
            sess->base_height   = rd.value("baseHeight",   1080);
            sess->output_width  = rd.value("outputWidth",  1920);
            sess->output_height = rd.value("outputHeight", 1080);
            sess->fps_num       = rd.value("fpsNumerator", 25);
            sess->fps_den       = rd.value("fpsDenominator", 1);
            printf("  ✓ Video actuelle: %dx%d -> %dx%d @ %d/%d fps\n",
                   sess->base_width, sess->base_height,
                   sess->output_width, sess->output_height,
                   sess->fps_num, sess->fps_den);
        }
        sess->state = STATE_APPLY_VIDEO_SETTINGS;
        lws_callback_on_writable(sess->wsi);
        break;

    case STATE_WAIT_VIDEO_SETTINGS:
        if (op != 7) break;
        printf("  ✓ FPS appliqué: %d fps\n", sess->config.fps_int);
        sess->state = next_after_video_settings(sess);
        lws_callback_on_writable(sess->wsi);
        break;

    case STATE_WAIT_STREAM_PARAM:
        if (op != 7) break;
        printf("  ✓ Stream param [%s/%s] = %s\n",
               sess->current_param->category, sess->current_param->name, sess->current_param->value);
        sess->current_param = sess->current_param->next;
        sess->state = sess->current_param ? STATE_APPLY_STREAM_PARAMS : next_after_stream_params(sess);
        lws_callback_on_writable(sess->wsi);
        break;

    case STATE_WAIT_REC_PARAM:
        if (op != 7) break;
        printf("  ✓ Rec param [%s/%s] = %s\n",
               sess->current_param->category, sess->current_param->name, sess->current_param->value);
        sess->current_param = sess->current_param->next;
        sess->state = sess->current_param ? STATE_APPLY_REC_PARAMS : next_after_rec_params(sess);
        lws_callback_on_writable(sess->wsi);
        break;

    case STATE_WAIT_START_STREAM:
        if (op != 7) break;
        printf("  ✓ Stream relancé\n");
        sess->state = next_after_start_stream(sess);
        if (sess->state == STATE_DONE) g_force_exit = 1;
        else lws_callback_on_writable(sess->wsi);
        break;

    case STATE_WAIT_START_RECORD:
        if (op != 7) break;
        printf("  ✓ Enregistrement relancé\n");
        sess->state = STATE_DONE;
        g_force_exit = 1;
        break;

    default: break;
    }
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
        char *msg = nullptr;
        switch (sess->state) {
        case STATE_STOP_STREAM:
            printf("[étape] Arrêt du stream…\n");
            msg = build_request("StopStream", ++sess->request_id, nullptr);
            sess->state = STATE_WAIT_STOP_STREAM;
            break;
        case STATE_STOP_RECORD:
            printf("[étape] Arrêt de l'enregistrement…\n");
            msg = build_request("StopRecord", ++sess->request_id, nullptr);
            sess->state = STATE_WAIT_STOP_RECORD;
            break;
        case STATE_GET_VIDEO_SETTINGS:
            printf("[étape] Lecture des paramètres vidéo courants…\n");
            msg = build_get_video_settings(++sess->request_id);
            sess->state = STATE_WAIT_GET_VIDEO_SETTINGS;
            break;
        case STATE_APPLY_VIDEO_SETTINGS:
            printf("[étape] SetVideoSettings FPS=%d\n", sess->config.fps_int);
            msg = build_set_video_settings(++sess->request_id, sess);
            sess->state = STATE_WAIT_VIDEO_SETTINGS;
            break;
        case STATE_APPLY_STREAM_PARAMS:
            if (!sess->current_param) sess->current_param = sess->config.stream_params;
            printf("[étape] Set stream %s/%s = %s\n",
                   sess->current_param->category, sess->current_param->name, sess->current_param->value);
            msg = build_set_profile_param(++sess->request_id, sess->current_param);
            sess->state = STATE_WAIT_STREAM_PARAM;
            break;
        case STATE_APPLY_REC_PARAMS:
            if (!sess->current_param) sess->current_param = sess->config.rec_params;
            printf("[étape] Set rec %s/%s = %s\n",
                   sess->current_param->category, sess->current_param->name, sess->current_param->value);
            msg = build_set_profile_param(++sess->request_id, sess->current_param);
            sess->state = STATE_WAIT_REC_PARAM;
            break;
        case STATE_START_STREAM:
            printf("[étape] Relance du stream…\n");
            msg = build_request("StartStream", ++sess->request_id, nullptr);
            sess->state = STATE_WAIT_START_STREAM;
            break;
        case STATE_START_RECORD:
            printf("[étape] Relance de l'enregistrement…\n");
            msg = build_request("StartRecord", ++sess->request_id, nullptr);
            sess->state = STATE_WAIT_START_RECORD;
            break;
        default: break;
        }
        if (msg) { ws_send(wsi, msg); free(msg); }
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

    default: break;
    }
    return 0;
}

static const struct lws_protocols protocols[] = {
    { "obs-websocket", callback_obs, 0, 65536, 0, nullptr, 0 },
    { nullptr, nullptr, 0, 0, 0, nullptr, 0 }
};

/* ── Usage ──────────────────────────────────────────────────────── */

static void usage(const char *prog)
{
    printf(
    "Usage: %s [options]\n\n"
    "Connexion:\n"
    "  -H <host>           Hôte OBS (défaut: localhost)\n"
    "  -p <port>           Port WebSocket (défaut: 4455)\n"
    "  -w <password>       Mot de passe\n\n"
    "Mode:\n"
    "  -m <mode>           stream | record | both (défaut: stream)\n\n"
    "Paramètres stream (SimpleOutput):\n"
    "  -b <bitrate>        Bitrate vidéo stream (kbps)\n\n"
    "Paramètres enregistrement (AdvOut / Custom FFmpeg):\n"
    "  --rec-bitrate <n>   FFVBitrate  - bitrate vidéo rec (kbps)\n"
    "  --rec-abitrate <n>  FFABitrate  - bitrate audio rec (kbps)\n"
    "  -e <encoder>        FFVEncoder  - encodeur vidéo (libx264, hevc_nvenc…)\n"
    "  --aencoder <enc>    FFAEncoder  - encodeur audio (aac, opus…)\n"
    "  -f <format>         FFFormat    - conteneur (mpegts, mp4, mkv…)\n"
    "  -u <url>            FFURL       - URL de sortie (srt://, udp://…)\n"
    "  -g <gop>            FFVGOPSize  - taille du GOP\n"
    "  --ff-custom <str>   FFVCustom   - params encodeur vidéo FFmpeg\n"
    "  --ff-acustom <str>  FFACustom   - params encodeur audio FFmpeg\n"
    "  --ff-mcustom <str>  FFMCustom   - params muxer FFmpeg\n"
    "  --rescale <WxH>     FFRescaleRes - résolution de rescale (active aussi FFRescale)\n"
    "  --to-file <bool>    FFOutputToFile - true/false\n"
    "  --sample-rate <n>   FFSamplingRate - fréquence audio (ex: 22050, 44100, 48000)\n"
    "  --ignore-compat <b> FFIgnoreCompat - ignorer compat format (true/false)\n"
    "  --audio-mixes <n>   FFAudioMixes   - masque pistes audio (1=piste1, 2=piste2…)\n\n"
    "Paramètres vidéo globaux (Video):\n"
    "  --fps <n>           FPSInt         - fréquence d'images (ex: 15, 25, 30, 50, 60)\n\n"
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
        {"rec-bitrate",   required_argument, 0, 'R'},
        {"rec-abitrate",  required_argument, 0, 'A'},
        {"aencoder",      required_argument, 0, 'E'},
        {"ff-custom",     required_argument, 0, 'C'},
        {"ff-acustom",    required_argument, 0, 'D'},
        {"ff-mcustom",    required_argument, 0, 'M'},
        {"rescale",       required_argument, 0, 'S'},
        {"to-file",       required_argument, 0, 'T'},
        {"sample-rate",   required_argument, 0, 'r'},
        {"ignore-compat", required_argument, 0, 'I'},
        {"audio-mixes",   required_argument, 0, 'X'},
        {"fps",           required_argument, 0, 'F'},
        {"help",          no_argument,       0, '?'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "b:m:H:p:w:e:f:u:g:?", long_opts, nullptr)) != -1) {
        switch (opt) {
        case 'H': g_session.config.host     = optarg; break;
        case 'p': g_session.config.port     = atoi(optarg); break;
        case 'w': g_session.config.password = optarg; break;
        case 'm':
            if      (!strcmp(optarg, "stream")) g_session.config.mode = MODE_STREAM;
            else if (!strcmp(optarg, "record")) g_session.config.mode = MODE_RECORD;
            else if (!strcmp(optarg, "both"))   g_session.config.mode = MODE_BOTH;
            else { fprintf(stderr, "Mode invalide: %s\n", optarg); return 1; }
            break;
        case 'b': param_append(&g_session.config.stream_params, param_new("SimpleOutput", "VBitrate",       optarg)); break;
        case 'R': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFVBitrate",           optarg)); break;
        case 'A': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFABitrate",           optarg)); break;
        case 'e': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFVEncoder",           optarg)); break;
        case 'E': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFAEncoder",           optarg)); break;
        case 'f': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFFormat",             optarg)); break;
        case 'u': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFURL",                optarg)); break;
        case 'g': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFVGOPSize",           optarg)); break;
        case 'C': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFVCustom",            optarg)); break;
        case 'D': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFACustom",            optarg)); break;
        case 'M': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFMCustom",            optarg)); break;
        case 'S':
            param_append(&g_session.config.rec_params, param_new("AdvOut", "FFRescale",    "true"));
            param_append(&g_session.config.rec_params, param_new("AdvOut", "FFRescaleRes", optarg));
            break;
        case 'T': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFOutputToFile",       optarg)); break;
        case 'r': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFSamplingRate",       optarg)); break;
        case 'I': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFIgnoreCompat",       optarg)); break;
        case 'X': param_append(&g_session.config.rec_params,    param_new("AdvOut", "FFAudioMixes",         optarg)); break;
        case 'F':
            g_session.config.fps_int = atoi(optarg);
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
    printf("  Host:          %s:%d\n", g_session.config.host, g_session.config.port);
    printf("  Mode:          %s\n", mode_str);
    printf("  Stream params: %d\n", param_count(g_session.config.stream_params));
    printf("  Rec params:    %d\n", param_count(g_session.config.rec_params));
    if (g_session.config.fps_int > 0)
        printf("  FPS cible:     %d\n", g_session.config.fps_int);
    printf("═══════════════════════════════════════════════════\n\n");

    for (param_change_t *p = g_session.config.stream_params; p; p = p->next)
        printf("  stream: %s/%s = %s\n", p->category, p->name, p->value);
    for (param_change_t *p = g_session.config.rec_params; p; p = p->next)
        printf("  rec:    %s/%s = %s\n", p->category, p->name, p->value);
    if (g_session.config.stream_params || g_session.config.rec_params) printf("\n");

    struct lws_context_creation_info ctx_info;
    memset(&ctx_info, 0, sizeof(ctx_info));
    ctx_info.port      = CONTEXT_PORT_NO_LISTEN;
    ctx_info.protocols = protocols;
    ctx_info.options   = LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;

    struct lws_context *context = lws_create_context(&ctx_info);
    if (!context) { fprintf(stderr, "✗ Impossible de créer le contexte LWS\n"); return 1; }

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
    if (!wsi) { fprintf(stderr, "✗ Impossible de se connecter\n"); lws_context_destroy(context); return 1; }

    while (!g_force_exit) lws_service(context, 100);

    lws_context_destroy(context);
    printf("\n✓ Terminé.\n");
    return 0;
}
