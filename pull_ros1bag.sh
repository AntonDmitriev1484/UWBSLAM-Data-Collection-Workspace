#!/usr/bin/env bash

# Check if an argument was provided
if [ $# -lt 1 ]; then
  echo "Usage: $0 <bagname>"
  exit 1
fi

BAGNAME=$1

ansible-playbook ansible/pull/pull.yml -e "file_paths=['collect/ros1/${BAGNAME}.bag']"
