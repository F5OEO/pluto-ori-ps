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
#   MQTT_HOST=broker ./bitrate_strategy.sh
#
# Dependencies: mosquitto-clients, jq
#
# ─────────────────────────────────────────────────────────────────────────────
# INPUT JSON  (topic: cmd/encoder/auto_bitrate)
# ─────────────────────────────────────────────────────────────────────────────
# {
#   "bitrate":   1000,              // REQUIRED — total budget in kbps
#
#   /* Pass-through to cmd/encoder (optional, override strategy defaults) */
#   "host":      "10.0.0.54",      // OBS host        (default: localhost)
#   "port":      4455,              // OBS WS port     (default: 4455)
#   "password":  "secret",         // OBS WS password
#   "mode":      "record",         // stream|record|both  (default: record)
#   "url":       "udp://230.0.0.5:10000?pkt_size=1316",   // FFURL
#   "format":    "mpegts",         // FFFormat        (default: mpegts)
#   "encoder":   "libx264",        // FFVEncoder      (default: libx264)
#   "fps":       25,                // frame rate      (default: per-tier)
#   "margin_factor": 0.90          // total headroom factor including TS overhead (default: 0.90)
# }
#
# ─────────────────────────────────────────────────────────────────────────────
# DEEP-OPTIMISED 16:9 CBR STRATEGY TIERS (Max Subjective Quality)
# ─────────────────────────────────────────────────────────────────────────────
# Budget      Resolution   Video BR        Audio BR  GOP (2s)  x264 Preset  sample_rate  channels  fps
# ──────────  ───────────  ──────────────  ────────  ────────  ───────────  ───────────  ────────  ───
# < 250 kbps  432x240      (Total*0.9)-16  16 kbps    20       slower       24000 Hz     mono      10
# 250–599     640x360      (Total*0.9)-32  32 kbps    30       slow         24000 Hz     mono      15
# 600–1499    960x540      (Total*0.9)-64  64 kbps    50       medium       44100 Hz     stereo    25
# 1500–3499   1280x720     (Total*0.9)-96  96 kbps    50       medium       48000 Hz     stereo    25
# ≥ 3500      1920x1080    (Total*0.9)-128 128 kbps   50       medium       48000 Hz     stereo    25
#
# Advanced x264 CBR Tuning applied:
# - nal-hrd=cbr + minrate/maxrate to force strict padding stuffing.
# - bufsize=video_bitrate (1s window) to prevent aggressive macroblocking on keyframes.
# - profile=high to enable 8x8 transform spatial optimization.
# - bf=3:b-adapt=2 to maximize compression efficiency on static areas.
# - rc-lookahead=40 to smoothly buffer and anticipate scene cuts.
# - aq-mode=1 to prevent severe color banding in flat gradients and dark zones.
# ─────────────────────────────────────────────────────────────────────────────

MQTT_HOST="${1:-${MQTT_HOST:-localhost}}"
MQTT_PORT="${2:-${MQTT_PORT:-1883}}"
TOPIC_IN="cmd/encoder/auto_bitrate"
TOPIC_OUT="cmd/encoder"

echo "[bitrate_strategy] Listening on mqtt://$MQTT_HOST:$MQTT_PORT/$TOPIC_IN"

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

    # ── Sélection de la stratégie par palier (16:9 Unifié & Presets Dynamiques) ──
    if [ "$bitrate" -lt 250 ]; then
        resolution="432x240";  audio_br=16; tier_fps=10; muxdelay="2000000"; preset_tier="slower"
    elif [ "$bitrate" -lt 600 ]; then
        resolution="640x360";  audio_br=32; tier_fps=15; muxdelay="1500000"; preset_tier="slow"
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

    # ── Audio à bas débit (Optimisation échantillonnage & canaux) ──────────
    sample_rate=48000
    audio_custom=""
    if [ "$audio_br" -le 32 ]; then
        sample_rate=24000
        audio_custom="-ac 1"
    elif [ "$audio_br" -le 64 ]; then
        sample_rate=44100
    fi

    # ── Calcul de la bande passante Vidéo (Marge brute de 10% pour le TS Mux) ──
    margin_override=$(jq_get '.margin_factor')
    if [ -n "$margin_override" ]; then
        margin="$margin_override"
    else
        margin="0.90" # 90% pour l'audio+vidéo, 10% réservés aux en-têtes MPEG-TS
    fi

    video_br=$(awk "BEGIN { printf \"%d\", ($bitrate * $margin) - $audio_br }")
    if [ "$video_br" -le 0 ]; then
        echo "  ERROR: bitrate too low ($bitrate kbps), video budget collapsed."
        continue
    fi
    
    muxrate=$(( bitrate * 1000 ))
    bufsize=$video_br

    # ── Agrégation des arguments avancés de libx264 ────────────────────────
    # Combine HRD CBR, le profil High, le preset de palier, le Lookahead, les B-Frames et l'AQ-mode
    x264_args="nal-hrd=cbr:profile=high:preset=${preset_tier}:bf=3:b-adapt=2:rc-lookahead=40:aq-mode=1"

    # ── Récupération des variables pass-through ──────────────────────────
    host=$(jq_get '.host');       [ -z "$host" ]     && host="localhost"
    port=$(jq_get '.port');       [ -z "$port" ]     && port="4455"
    password=$(jq_get '.password')
    mode=$(jq_get '.mode');       [ -z "$mode" ]     && mode="record"
    url=$(jq_get '.url')
    format=$(jq_get '.format');   [ -z "$format" ]   && format="mpegts"
    encoder=$(jq_get '.encoder'); [ -z "$encoder" ]  && encoder="libx264"

    # ── Construction du Payload JSON (Avec injection des paramètres psycho-visuels) ──
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
        --arg  audio_encoder "aac"        \
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
    echo "  Strategy: ${bitrate}kbps → video=${video_br}k audio=${audio_br}k/${sample_rate}Hz/${channels_str} res=${resolution} fps=${fps} gop=${gop} preset=${preset_tier} margin=${margin}"
    echo "  Publishing to $TOPIC_OUT: $payload"

    mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "$TOPIC_OUT" -m "$payload"
    echo "  EXIT: $?"
done
echo "[bitrate_strategy] disconnected, reconnecting in 5s…"
sleep 5
done