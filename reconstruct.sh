#!/bin/bash

# Enable Tracing for Debugging.
#set -x

#$1 = Project Path
#$2 = Input Video Path for vid2frame
#$3 = 'sparse' or 'dense'

PARAMETER_COUNT=3
PROJECT_PATH=$1
IMAGE_DATA_PATH="$(readlink -m $PROJECT_PATH/image_data)"
DATABASE_PATH="$(readlink -m $PROJECT_PATH/database.db)"

# Check if call is legal.
echo "$#"
if [ "$#" != "$PARAMETER_COUNT" ]; then
  echo "Insufficient Parameters supplied. This script requires $PARAMETER_COUNT mandatory parameter(s)"
  exit 1
fi

#
# if [ "$(ls -A $IMAGE_DATA_PATH)" ]; then
#   echo "Images(s) found in $IMAGE_DATA_PATH. Proceeding..."
# else
#   echo "$IMAGE_DATA_PATH is empty"
#   exit 2
# fi

# Build Required Binaries

if [[ -x "vid2frames" ]]; then
  echo "vid2frames located and is executable. Proceeding..."
else
  echo "No binary for vid2frames found. Building one now..."
  g++ vid2frames.cpp -o $PROJECT_PATH/vid2frames `pkg-config opencv --cflags --libs`
fi

# Create relevant directories

echo "Project path is $PROJECT_PATH"

# Create Frame Data from Video.
if [ ! -d "$IMAGE_DATA_PATH" ]; then
  mkdir $IMAGE_DATA_PATH
fi
./vid2frames $2 $IMAGE_DATA_PATH 3

/usr/local/bin/feature_extractor \
--image_path=$IMAGE_DATA_PATH \
--database_path=$DATABASE_PATH \

/usr/local/bin/exhaustive_matcher \
--database_path=$DATABASE_PATH \

if [ "$3" = "sparse" ]; then \
  SPARSE_PATH="$(readlink -m $PROJECT_PATH/sparse)"
  mkdir $SPARSE_PATH

  echo "Initiating Sparse Reconstruction..."

  /usr/local/bin/mapper \
  --image_path=$IMAGE_DATA_PATH \
  --database_path=$DATABASE_PATH \
  --export_path=$SPARSE_PATH \

  echo "Sparse Reconstruction Complete."

elif [ "$3" = "dense" ]; then \

  DENSE_PATH="$(readlink -m $PROJECT_PATH/dense)"
  mkdir $DENSE_PATH

  echo "Initiating Dense Reconstruction..."

  ./src/exe/image_undistorter \
  --image_path $PROJECT_PATH/images \
  --input_path $PROJECT_PATH/sparse/0 \
  --output_path $PROJECT_PATH/dense \
  --output_type COLMAP \
  --max_image_size 2000

  ./exe/dense_stereo \
  --workspace_path $PROJECT_PATH/dense \
  --workspace_format COLMAP \
  --DenseStereo.geom_consistency true

  ./exe/dense_fuser \
  --workspace_path $PROJECT_PATH/dense \
  --workspace_format COLMAP \
  --input_type geometric \
  --output_path $PROJECT_PATH/dense/fused.ply

  ./src/exe/dense_mesher \
  --input_path $PROJECT_PATH/dense/fused.ply \
  --output_path $PROJECT_PATH/dense/meshed.ply

  echo "Dense Reconstruction Complete."

else
  echo "Invalid Reconstruction Specifier. Exiting."
  exit 3

fi
