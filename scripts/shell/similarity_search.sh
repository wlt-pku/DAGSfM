DATASET_PATH=$1
OUTPUT_PATH=$2
MOST_SIMILAR_IMAGES_NUM=$3
MIRROR_PATH=$4

# MIRROR_PATH=/home/chenyu/Projects/Disco/lib/mirror
MODEL_PATH=$MIRROR_PATH/data/model/model.ckpt

ls -d $DATASET_PATH/images/* > $DATASET_PATH/image_list.txt

# 1. generate the .npy features.
python $MIRROR_PATH/retrieval/inference.py \
--img_list $DATASET_PATH/image_list.txt \
--ckpt_step 720000 \
--img_size 896 \
--net resnet-50 \
--pool MAX \
--rmac_step 1,3,5 \
--ckpt_path $MODEL_PATH

# 2. output all the absolute paths of feature files (*.npy) to a file and query
mkdir $DATASET_PATH/dl_features
mv $DATASET_PATH/images/*.npy $DATASET_PATH/dl_features
ls -d $DATASET_PATH/dl_features/*.npy > $DATASET_PATH/feature_list.txt

# 3. Deep query
python $MIRROR_PATH/retrieval/deep_query.py \
--feature_list $DATASET_PATH/feature_list.txt \
--out_dir $OUTPUT_PATH \
--top $MOST_SIMILAR_IMAGES_NUM \
--out_dim 256

# 4. Remove redundency files.
# rm $DATASET_PATH/image_list.txt
rm $DATASET_PATH/feature_list.txt
rm -rf $DATASET_PATH/dl_features