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
#   "encoder":   "libx264"         // FFVEncoder      (default: libx264)
# }
#
# ─────────────────────────────────────────────────────────────────────────────
# BITRATE STRATEGY TIERS
# ─────────────────────────────────────────────────────────────────────────────
# Budget      Resolution   Video    Audio   GOP  muxdelay  sample_rate
# ──────────  ───────────  ───────  ──────  ───  ────────  ───────────
# < 200 kbps  352x288      B-16-8   16kbps  100   2.0s     22050 Hz
# 200–499     480x360      B-32-16  32kbps   75   1.5s     44100 Hz
# 500–1499    720x576      B-64-32  64kbps   50   1.0s     44100 Hz
# 1500–3999   1280x720     B-96-64  96kbps   25   0.7s     48000 Hz
# ≥ 4000      1920x1080    B-128-128 128kbps 25   0.5s     48000 Hz
#
# muxrate = bitrate * 1000 bps  (CBR stuffing fills to exact budget)
# video CBR enforced via minrate=maxrate=video_bitrate, bufsize=2×video_bitrate
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

    # ── Select tier ──────────────────────────────────────────────────────
    if [ "$bitrate" -lt 200 ]; then
        resolution="352x288"; audio_br=16;  gop=100; muxdelay="2.0"; sample_rate=22050; overhead=24
    elif [ "$bitrate" -lt 500 ]; then
        resolution="480x360"; audio_br=32;  gop=75;  muxdelay="1.5"; sample_rate=44100; overhead=48
    elif [ "$bitrate" -lt 1500 ]; then
        resolution="720x576"; audio_br=64;  gop=50;  muxdelay="1.0"; sample_rate=44100; overhead=96
    elif [ "$bitrate" -lt 4000 ]; then
        resolution="1280x720"; audio_br=96; gop=25;  muxdelay="0.7"; sample_rate=48000; overhead=160
    else
        resolution="1920x1080"; audio_br=128; gop=25; muxdelay="0.5"; sample_rate=48000; overhead=256
    fi

    video_br=$(( bitrate - audio_br - overhead ))
    if [ "$video_br" -le 0 ]; then
        echo "  ERROR: bitrate too low ($bitrate kbps), minimum is $(( audio_br + overhead + 1 )) kbps"
        continue
    fi
    muxrate=$(( bitrate * 1000 ))
    bufsize=$(( video_br * 2 ))

    # ── Read pass-through fields ──────────────────────────────────────────
    host=$(jq_get '.host');       [ -z "$host" ]     && host="localhost"
    port=$(jq_get '.port');       [ -z "$port" ]     && port="4455"
    password=$(jq_get '.password')
    mode=$(jq_get '.mode');       [ -z "$mode" ]     && mode="record"
    url=$(jq_get '.url')
    format=$(jq_get '.format');   [ -z "$format" ]   && format="mpegts"
    encoder=$(jq_get '.encoder'); [ -z "$encoder" ]  && encoder="libx264"

    # ── Build output JSON ─────────────────────────────────────────────────
    payload=$(jq -n \
        --arg  host         "$host"       \
        --argjson port      "$port"       \
        --arg  password     "$password"   \
        --arg  mode         "$mode"       \
        --arg  format       "$format"     \
        --arg  url          "$url"        \
        --arg  encoder      "$encoder"    \
        --argjson video_br  "$video_br"   \
        --arg  video_custom "minrate=${video_br}k maxrate=${video_br}k bufsize=${bufsize}k" \
        --argjson audio_br  "$audio_br"   \
        --arg  audio_encoder "aac"        \
        --argjson sample_rate "$sample_rate" \
        --argjson gop       "$gop"        \
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
            gop:           $gop,
            rescale:       $rescale,
            mux_custom:    $mux_custom
        } | if $password == "" then del(.password) else . end
          | if $url == ""      then del(.url)      else . end'
    )

    echo "  Strategy: ${bitrate}kbps → video=${video_br}k audio=${audio_br}k res=${resolution} gop=${gop} muxdelay=${muxdelay}s"
    echo "  Publishing to $TOPIC_OUT: $payload"

    mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "$TOPIC_OUT" -m "$payload"
    echo "  EXIT: $?"
done
echo "[bitrate_strategy] disconnected, reconnecting in 5s…"
sleep 5
done
