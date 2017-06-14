#!/bin/bash

# Enable Tracing for Debugging.
set -x

PARAMETER_COUNT=2
PROJECT_PATH=$1
IMAGE_PATH="$(readlink -m $PROJECT_PATH/images)"
DATABASE_PATH="$(readlink -m $PROJECT_PATH/database.db)"

if [ "$#" != "$PARAMETER_COUNT" ]; then
	echo "Insufficient Parameters supplied. This script requires $PARAMETER_COUNT mandatory parameter(s)"
	exit 1
fi

if [ "$(ls -A $IMAGE_PATH)" ]; then
     echo "Images(s) found in $IMAGE_PATH. Proceeding."
else
    echo "$IMAGE_PATH is empty"
		exit 2
fi

# Create relevant directories

echo "Project path is $PROJECT_PATH"

/usr/local/bin/feature_extractor \
--image_path=$IMAGE_PATH \
--database_path=$DATABASE_PATH \

/usr/local/bin/exhaustive_matcher \
--database_path=$DATABASE_PATH \

if [ "$2" = "sparse" ]; then \

SPARSE_PATH="$(readlink -m $PROJECT_PATH/sparse)"
mkdir $SPARSE_PATH
/usr/local/bin/mapper \
--image_path=$IMAGE_PATH \
--database_path=$DATABASE_PATH \
--export_path=$SPARSE_PATH \

#~/3DU/file2cloud

elif [ "$2" = "dense" ]; then \
	echo "Dense Not Ready"

else
	echo "Invalid Reconstruction Specifier. Exiting."
	exit 3

fi
