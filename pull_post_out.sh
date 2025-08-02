#!/usr/bin/env bash

# Check if an argument was provided
if [ $# -lt 1 ]; then
  echo "Usage: $0 <trialname>"
  exit 1
fi

TRIAL=$1

ansible-playbook ansible/pull/pull.yml -e "file_paths=['post/out/${TRIAL}_post']" -e "dir=true"
