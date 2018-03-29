#!/bin/bash
set -e
PROJECT="pillserial"
echo "==== Building....===="
docker build -t $PROJECT .
echo "==== Copying firmware....===="
docker run -v $PWD:/mnt $PROJECT bash -c "cp -v /home/$PROJECT/code/src/*.bin /mnt"
