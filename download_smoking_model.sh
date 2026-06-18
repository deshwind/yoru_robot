#!/usr/bin/env bash
# Downloads the pretrained models used by the laptop CCTV camera's smoking
# detector (yolo_detector_node 'smoking_model_path'):
#
#   cigarette_yolo.pt  huggingface.co/Enos-123/smoking-detection
#                      YOLOv11-M, class 'cigarette' -> 'cigarette' violation
#   smoke_yolo.pt      huggingface.co/kittendev/YOLOv8m-smoke-detection
#                      YOLOv8-M, class 'smoke' -> 'smoke_vapour' (smoke at the
#                      mouth confirms on its own in event_confirmation_node)
#
# Run this ON THE MACHINE THAT RUNS THE CCTV CAMERA (the server/laptop), then
# restart with ./start_server.sh
set -e
cd "$(dirname "$0")"

fetch() {  # url  output
    echo "Downloading $2 ..."
    curl -fL --retry 3 -o "$2.part" "$1"
    mv -f "$2.part" "$2"
    SIZE=$(stat -c%s "$2" 2>/dev/null || stat -f%z "$2" 2>/dev/null || echo "?")
    echo "  done: $2 ($SIZE bytes)"
}

fetch "https://huggingface.co/Enos-123/smoking-detection/resolve/main/best.pt" "cigarette_yolo.pt"
fetch "https://huggingface.co/kittendev/YOLOv8m-smoke-detection/resolve/main/best.pt" "smoke_yolo.pt"

echo
echo "Smoking detection also needs the YOLO stack on this machine:"
echo "    pip install 'ultralytics>=8.3'    # >=8.3 for the YOLOv11 weights"
echo "Then start the server:  ./start_server.sh"
