from robopy.utils.h5_handler import H5Handler

if __name__ == "__main__":
    # H5ファイルのパス
    h5_file_path = "path/to/your/hierarchical_data.h5"

    # H5ファイルの情報を取得して表示
    file_info = H5Handler.get_info(h5_file_path)
    print("H5 File Information:")
    for key, value in file_info.items():
        print(f"  {key}: shape={value['shape']}, dtype={value['dtype']}")

    # 階層的なデータを読み込む
    hierarchical_data = H5Handler.load_hierarchical(h5_file_path)
    print("\nLoaded Hierarchical Data:")
    for category, datasets in hierarchical_data.items():
        for name, data in datasets.items():
            print(f"  {category}/{name}: shape={data.shape}, dtype={data.dtype}")
