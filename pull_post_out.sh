#!/usr/bin/env bash

# Check if an argument was provided
if [ $# -lt 2 ]; then
  echo "Usage: $0 <trialname> <host>"
  exit 1
fi

TRIAL=$1
HOST=$2

ansible-playbook ansible/pull/pull.yml -e "file_paths=['post/out/${TRIAL}_post']" -e "dir=true" -e "remote_host=${HOST}"
