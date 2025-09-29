#!/usr/bin/env bash

# Check if an argument was provided
if [ $# -lt 1 ]; then
  echo "Usage: $0 <trialname>"
  exit 1
fi

TRIAL=$1

ansible-playbook ansible/multi_pull/multi_pull.yml -e "file_paths=['post/out/${TRIAL}_post']" -e "dir=true" -e "multi=true" -e "basename=${TRIAL}_post" --ask-become-pass
cd post/utils/
python3 merge_all_json.py "${TRIAL}"