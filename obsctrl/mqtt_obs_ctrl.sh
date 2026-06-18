#!/bin/bash
#
# mqtt_obs_ctrl.sh
#
# Listens on MQTT topic cmd/encoder for JSON messages and launches
# obs_stream_control with the corresponding parameters.
#
# Usage:
#   ./mqtt_obs_ctrl.sh [mqtt_host [mqtt_port]]
#   MQTT_HOST=broker MQTT_PORT=1883 ./mqtt_obs_ctrl.sh
#
# Dependencies: mosquitto-clients, jq
#
# ─────────────────────────────────────────────────────────────────────────────
# JSON MESSAGE FORMAT
# ─────────────────────────────────────────────────────────────────────────────
# Publish to topic:  cmd/encoder
# All fields are optional except where noted.
#
# {
#   /* OBS WebSocket connection */
#   "host":           "10.0.0.54",   // OBS host          (default: localhost)
#   "port":           4455,           // OBS WS port        (default: 4455)
#   "password":       "secret",       // OBS WS password    (default: "")
#
#   /* Control mode */
#   "mode":           "record",       // stream | record | both  (default: record)
#
#   /* Output container / destination */
#   "format":         "mpegts",       // FFFormat  — mpegts, mp4, mkv, …
#   "url":            "udp://230.0.0.5:10000?pkt_size=1316",  // FFURL
#   "to_file":        false,          // FFOutputToFile — true writes to disk path
#
#   /* Video */
#   "video_encoder":  "libx264",      // FFVEncoder — libx264, libx265, hevc_nvenc, …
#   "video_bitrate":  500,            // FFVBitrate (kbps)
#   "gop":            50,             // FFVGOPSize
#   "rescale":        "720x576",      // FFRescaleRes — also sets FFRescale=true
#   "video_custom":   "minrate=500k maxrate=500k bufsize=1000k",  // FFVCustom
#
#   /* Audio */
#   "audio_encoder":  "aac",          // FFAEncoder — aac, opus, mp2, …
#   "audio_bitrate":  16,             // FFABitrate (kbps)
#   "sample_rate":    48000,          // FFSamplingRate — 22050 | 44100 | 48000
#   "audio_mixes":    1,              // FFAudioMixes — bitmask: 1=track1 2=track2 …
#   "audio_custom":   "",             // FFACustom
#
#   /* Muxer */
#   "mux_custom":     "muxrate=1000000 muxdelay=1.5",  // FFMCustom
#
#   /* Misc */
#   "ignore_compat":  false,          // FFIgnoreCompat — bypass OBS format check
#
#   /* Stream-only (SimpleOutput, used when mode=stream or mode=both) */
#   "stream_bitrate": 6000            // SimpleOutput/VBitrate (kbps)
# }
#
# Example publish:
#   mosquitto_pub -h localhost -t cmd/encoder -m '{
#     "host":"10.0.0.54","password":"tezukadvb","mode":"record",
#     "format":"mpegts","url":"udp://230.0.0.5:10000?pkt_size=1316",
#     "video_encoder":"libx264","video_bitrate":500,
#     "video_custom":"minrate=500k maxrate=500k bufsize=1000k",
#     "audio_encoder":"aac","audio_bitrate":16,
#     "mux_custom":"muxrate=1000000 muxdelay=1.5"
#   }'
# ─────────────────────────────────────────────────────────────────────────────

MQTT_HOST="${1:-${MQTT_HOST:-localhost}}"
MQTT_PORT="${2:-${MQTT_PORT:-1883}}"
MQTT_TOPIC="cmd/encoder"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
OBS_CTRL="$SCRIPT_DIR/obs_stream_control"

echo "[mqtt_obs_ctrl] Listening on mqtt://$MQTT_HOST:$MQTT_PORT/$MQTT_TOPIC"

while true; do
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "$MQTT_TOPIC" | while read -r msg; do
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Received: $msg"

    if ! echo "$msg" | jq . > /dev/null 2>&1; then
        echo "  ERROR: invalid JSON, skipping"
        continue
    fi

    jq_get() { echo "$msg" | jq -r "$1 // empty"; }

    CMD=("$OBS_CTRL")

    # Connection
    host=$(jq_get '.host')
    port=$(jq_get '.port')
    password=$(jq_get '.password')
    [ -n "$host" ]     && CMD+=(-H "$host")
    [ -n "$port" ]     && CMD+=(-p "$port")
    [ -n "$password" ] && CMD+=(-w "$password")

    # Mode
    mode=$(jq_get '.mode')
    [ -n "$mode" ] && CMD+=(-m "$mode")

    # Output
    format=$(jq_get '.format')
    [ -n "$format" ] && CMD+=(-f "$format")

    url=$(jq_get '.url')
    [ -n "$url" ] && CMD+=(-u "$url")

    to_file=$(jq_get '.to_file')
    [ -n "$to_file" ] && CMD+=(--to-file "$to_file")

    # Video
    video_encoder=$(jq_get '.video_encoder')
    [ -n "$video_encoder" ] && CMD+=(-e "$video_encoder")

    video_bitrate=$(jq_get '.video_bitrate')
    [ -n "$video_bitrate" ] && CMD+=(--rec-bitrate "$video_bitrate")

    gop=$(jq_get '.gop')
    [ -n "$gop" ] && CMD+=(-g "$gop")

    rescale=$(jq_get '.rescale')
    [ -n "$rescale" ] && CMD+=(--rescale "$rescale")

    video_custom=$(jq_get '.video_custom')
    [ -n "$video_custom" ] && CMD+=(--ff-custom "$video_custom")

    # Audio
    audio_encoder=$(jq_get '.audio_encoder')
    [ -n "$audio_encoder" ] && CMD+=(--aencoder "$audio_encoder")

    audio_bitrate=$(jq_get '.audio_bitrate')
    [ -n "$audio_bitrate" ] && CMD+=(--rec-abitrate "$audio_bitrate")

    sample_rate=$(jq_get '.sample_rate')
    [ -n "$sample_rate" ] && CMD+=(--sample-rate "$sample_rate")

    audio_mixes=$(jq_get '.audio_mixes')
    [ -n "$audio_mixes" ] && CMD+=(--audio-mixes "$audio_mixes")

    audio_custom=$(jq_get '.audio_custom')
    [ -n "$audio_custom" ] && CMD+=(--ff-acustom "$audio_custom")

    # Muxer
    mux_custom=$(jq_get '.mux_custom')
    [ -n "$mux_custom" ] && CMD+=(--ff-mcustom "$mux_custom")

    # Misc
    ignore_compat=$(jq_get '.ignore_compat')
    [ -n "$ignore_compat" ] && CMD+=(--ignore-compat "$ignore_compat")

    # Stream-only
    stream_bitrate=$(jq_get '.stream_bitrate')
    [ -n "$stream_bitrate" ] && CMD+=(-b "$stream_bitrate")

    echo "  CMD: ${CMD[*]}"
    "${CMD[@]}"
    echo "  EXIT: $?"
done
echo "[mqtt_obs_ctrl] disconnected, reconnecting in 5s…"
sleep 5
done
