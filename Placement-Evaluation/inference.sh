




# test late fusion
python v2xvit/tools/inference.py --model_dir v2xvit/logs/point_pillar_late_fusion_2022_09_05_01_30_43 --fusion_method late --test_dir v2xset/for_test/validate_v1

# test opv2v
python v2xvit/tools/inference.py --model_dir v2xvit/logs/point_pillar_opv2v_2022_09_02_05_12_28 --fusion_method intermediate --test_dir v2xset/for_test/validate_v1

# test v2vnet
python v2xvit/tools/inference.py --model_dir v2xvit/logs/point_pillar_v2vnet_2022_09_05_13_19_38 --fusion_method intermediate --test_dir v2xset/for_test/validate_v1

# test early fusion
python v2xvit/tools/inference.py --model_dir v2xvit/logs/point_pillar_early_fusion_2022_09_05_01_18_54 --fusion_method early --test_dir v2xset/for_test/validate_v1






