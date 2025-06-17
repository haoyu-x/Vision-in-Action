import zarr
import numpy as np

def save_to_zarr(save_path, data):
    """Save dictionary data to a Zarr file."""
    store = zarr.ZipStore(save_path, mode='w')
    root = zarr.group(store)

    # Save each item in the data dictionary
    for key, value in data.items():
        if isinstance(value, list) and all(isinstance(arr, np.ndarray) for arr in value):
            # Stack arrays if they are of consistent shapes, otherwise store as individual datasets
            if all(arr.shape == value[0].shape for arr in value):
                # Stack and save if shapes are consistent
                data_arr = np.stack(value)
                root.array(
                    key,
                    data_arr,
                    dtype=data_arr.dtype,
                    compressor=zarr.Blosc(cname='zstd', clevel=3, shuffle=zarr.Blosc.SHUFFLE)
                )
            else:
                # Save as individual datasets if shapes vary
                group = root.create_group(key)
                for i, arr in enumerate(value):
                    group.array(
                        f"{key}_{i}",
                        arr,
                        dtype=arr.dtype,
                        compressor=zarr.Blosc(cname='zstd', clevel=3, shuffle=zarr.Blosc.SHUFFLE)
                    )
        else:
            # If the item is not a list or doesn't contain numpy arrays, save directly
            root.array(
                key,
                value,
                dtype=value[0].dtype if isinstance(value[0], np.ndarray) else None,
                compressor=zarr.Blosc(cname='zstd', clevel=3, shuffle=zarr.Blosc.SHUFFLE)
            )

    store.close()
    print(f"Saved data to {save_path}")
