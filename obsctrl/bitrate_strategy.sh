#!/bin/bash
#
# bitrate_strategy.sh
#
# Listens on cmd/encoder/auto_bitrate for a total available bitrate (kbps)
# and publishes an optimised cmd/encoder JSON message with the best
# resolution, video bitrate, audio bitrate and PCR/PTS timing for that budget.
#
# Usage:
#   ./bitrate_strategy.sh [mqtt_host [mqtt_port]]
#
# Dependencies: mosquitto-clients, jq
# ─────────────────────────────────────────────────────────────────────────────

MQTT_HOST="${1:-${MQTT_HOST:-localhost}}"
MQTT_PORT="${2:-${MQTT_PORT:-1883}}"
TOPIC_IN="cmd/encoder/auto_bitrate"
TOPIC_OUT="cmd/encoder"

# ⚠️ CONFIGURATION CRITIQUE POUR LE BAS DÉBIT AUDIO :
# - "false" : Permet d'utiliser 'libopus' sous les 600 kbps (Qualité incroyable à 16/32k).
# - "true"  : Impose l'AAC partout, forçant un plancher de 64k pour éviter la destruction du son.
FORCE_AAC="true"

echo "[bitrate_strategy] Listening on mqtt://$MQTT_HOST:$MQTT_PORT/$TOPIC_IN (FORCE_AAC=$FORCE_AAC)"

while true; do
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "$TOPIC_IN" | while read -r msg; do
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Received: $msg"

    if ! echo "$msg" | jq . > /dev/null 2>&1; then
        echo "  ERROR: invalid JSON, skipping"
        continue
    fi

    jq_get() { echo "$msg" | jq -r "$1 // empty"; }

    bitrate=$(jq_get '.bitrate')
    if [ -z "$bitrate" ]; then
        echo "  ERROR: missing 'bitrate' field, skipping"
        continue
    fi

    # ── Sélection de la stratégie par palier (16:9 Unifié & Presets) ──────
    if [ "$bitrate" -lt 250 ]; then
        resolution="432x240";  audio_br=16; tier_fps=10; muxdelay="2000000"; preset_tier="slower"
        [ "$FORCE_AAC" = "true" ] && audio_br=64 # Plancher de secours pour l'AAC-LC
    elif [ "$bitrate" -lt 600 ]; then
        resolution="640x360";  audio_br=32; tier_fps=15; muxdelay="1500000"; preset_tier="slow"
        [ "$FORCE_AAC" = "true" ] && audio_br=64 # Plancher de secours pour l'AAC-LC
    elif [ "$bitrate" -lt 1500 ]; then
        resolution="960x540";  audio_br=64; tier_fps=25; muxdelay="1000000"; preset_tier="medium"
    elif [ "$bitrate" -lt 3500 ]; then
        resolution="1280x720"; audio_br=96; tier_fps=25; muxdelay="700000";  preset_tier="medium"
    else
        resolution="1920x1080"; audio_br=128; tier_fps=25; muxdelay="500000"; preset_tier="medium"
    fi

    # ── Application des règles FPS et GOP (GOP dynamique de 2 secondes) ────
    fps=$(jq_get '.fps')
    [ -z "$fps" ] && fps="$tier_fps"
    gop=$(( fps * 2 ))

    # ── Intelligence Audio (Routage dynamique Opus vs AAC-LC) ──────────────
    if [ "$audio_br" -le 32 ] && [ "$FORCE_AAC" != "true" ]; then
        audio_encoder="libopus"
        sample_rate=48000 # Opus excelle à 48kHz, même à 16 kbps grâce à son architecture hybride
        audio_custom="-ac 1"
    else
        audio_encoder="aac"
        audio_custom=""
        if [ "$audio_br" -le 32 ]; then
            sample_rate=24000
            audio_custom="-ac 1"
        elif [ "$audio_br" -le 64 ]; then
            sample_rate=44100
            audio_custom="-ac 1" # Forcer le mono à 64k pour maximiser l'AAC-LC
        else
            sample_rate=48000
        fi
    fi

    # ── Calcul de la bande passante Vidéo (Marge brute de 10% pour le TS Mux) ──
    margin_override=$(jq_get '.margin_factor')
    if [ -n "$margin_override" ]; then
        margin="$margin_override"
    else
        margin="0.85"
    fi

    video_br=$(awk "BEGIN { printf \"%d\", ($bitrate * $margin) - $audio_br }")
    if [ "$video_br" -le 0 ]; then
        echo "  ERROR: bitrate too low ($bitrate kbps), video budget collapsed."
        continue
    fi
    
    muxrate=$(( bitrate * 1000 ))
    bufsize=$video_br

    # ── Configuration x264 Avancée ────────────────────────────────────────
    x264_args="nal-hrd=cbr:profile=high:preset=${preset_tier}:bf=3:b-adapt=2:rc-lookahead=40:aq-mode=1"

    # ── Récupération des variables pass-through ──────────────────────────
    host=$(jq_get '.host');       [ -z "$host" ]     && host="localhost"
    port=$(jq_get '.port');       [ -z "$port" ]     && port="4455"
    password=$(jq_get '.password')
    mode=$(jq_get '.mode');       [ -z "$mode" ]     && mode="record"
    url=$(jq_get '.url')
    format=$(jq_get '.format');   [ -z "$format" ]   && format="mpegts"
    encoder=$(jq_get '.encoder'); [ -z "$encoder" ]  && encoder="libx264"

    # ── Construction du Payload JSON ─────────────────────────────────────
    payload=$(jq -cn \
        --arg  host         "$host"       \
        --argjson port      "$port"       \
        --arg  password     "$password"   \
        --arg  mode         "$mode"       \
        --arg  format       "$format"     \
        --arg  url          "$url"        \
        --arg  encoder      "$encoder"    \
        --argjson video_br  "$video_br"   \
        --arg  video_custom "minrate=${video_br}k maxrate=${video_br}k bufsize=${bufsize}k -x264-params ${x264_args}" \
        --argjson audio_br  "$audio_br"   \
        --arg  audio_encoder "$audio_encoder" \
        --argjson sample_rate "$sample_rate" \
        --arg  audio_custom "$audio_custom" \
        --argjson gop       "$gop"        \
        --argjson fps       "$fps"        \
        --arg  rescale      "$resolution" \
        --arg  mux_custom   "muxrate=$muxrate muxdelay=$muxdelay" \
        '{
            host:          $host,
            port:          $port,
            password:      $password,
            mode:          $mode,
            format:        $format,
            url:           $url,
            video_encoder: $encoder,
            video_bitrate: $video_br,
            video_custom:  $video_custom,
            audio_encoder: $audio_encoder,
            audio_bitrate: $audio_br,
            sample_rate:   $sample_rate,
            audio_custom:  $audio_custom,
            gop:           $gop,
            fps:           $fps,
            rescale:       $rescale,
            mux_custom:    $mux_custom
        } | if $password    == "" then del(.password)    else . end
          | if $url         == "" then del(.url)         else . end
          | if $audio_custom == "" then del(.audio_custom) else . end'
    )

    channels_str="${audio_custom:+"mono"}"
    [ -z "$channels_str" ] && channels_str="stereo"
    echo "  Strategy: ${bitrate}kbps → video=${video_br}k audio=${audio_br}k via [${audio_encoder}]/${sample_rate}Hz/${channels_str} res=${resolution} fps=${fps}"
    echo "  Publishing to $TOPIC_OUT: $payload"

    mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "$TOPIC_OUT" -m "$payload"
    echo "  EXIT: $?"
done
echo "[bitrate_strategy] disconnected, reconnecting in 5s…"
sleep 5
done