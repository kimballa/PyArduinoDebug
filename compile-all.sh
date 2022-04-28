# (c) Copyright 2022 Aaron Kimball
#
# Compile PyArduinoDebug library for all supported architectures.

read -r -d '' BOARDS << EOL
arduino:avr:uno
arduino:avr:leonardo
adafruit:samd:adafruit_feather_m4
EOL

set -e
for board in ${BOARDS}; do
  echo -e "\033[1m*** Compiling for Arduino architecture: ${board}\033[0m"
  BOARD=${board} make clean install
  echo ""
  echo ""
done
