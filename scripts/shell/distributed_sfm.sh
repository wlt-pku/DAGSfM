DATASET_PATH=$1
num_images_ub=$2
log_folder=$3
CONFIG_FILE_PATH=$4

/home/chenyu/Projects/Disco/build/src/exe/colmap distributed_mapper \
$DATASET_PATH/$log_folder \
--database_path=$DATASET_PATH/database.db \
--image_path=$DATASET_PATH/images \
--output_path=$DATASET_PATH/$log_folder \
--config_file_name=$CONFIG_FILE_PATH/config.txt \
--num_workers=8 \
--distributed=1 \
--repartition=0 \
--num_images=100 \
--script_path=/home/chenyu/Projects/Disco/scripts/shell/similarity_search.sh \
--dataset_path=$DATASET_PATH \
--output_dir=$DATASET_PATH/$log_folder \
--mirror_path=/home/chenyu/Projects/Disco/lib/mirror \
--assign_cluster_id=0 \
--write_binary=1 \
--retriangulate=0 \
--final_ba=1 \
--select_tracks_for_bundle_adjustment=1 \
--long_track_length_threshold=10 \
--graph_dir=$DATASET_PATH/$log_folder \
--num_images_ub=$num_images_ub \
--completeness_ratio=0.7 \
--relax_ratio=1.3 \
--cluster_type=SPECTRA #SPECTRA #NCUT COMMUNITY_DETECTION #
# --max_num_cluster_pairs=$max_num_cluster_pairs \
# --image_overlap=$image_overlap \
