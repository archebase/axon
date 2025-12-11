#ifndef LANCE_BRIDGE_H
#define LANCE_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Error codes
#define LANCE_SUCCESS 0
#define LANCE_ERROR_INVALID_PATH -1
#define LANCE_ERROR_INVALID_SCHEMA -2
#define LANCE_ERROR_DATASET -3
#define LANCE_ERROR_ARROW -4
#define LANCE_ERROR_IO -5
#define LANCE_ERROR_INVALID_HANDLE -6

// Forward declarations for Arrow C Data Interface
struct ArrowArray;
struct ArrowSchema;

/**
 * Create or open a Lance dataset
 * 
 * @param path Path to the dataset (null-terminated C string)
 * @param schema_ptr Pointer to Arrow schema (FFI_ArrowSchema)
 * @return Dataset handle (positive int64_t) on success, or 0 on error
 */
int64_t create_or_open_dataset(const char* path, struct ArrowSchema* schema_ptr);

/**
 * Write a RecordBatch to a Lance dataset
 * 
 * @param dataset_handle Handle returned by create_or_open_dataset
 * @param array_ptr Pointer to Arrow array (FFI_ArrowArray)
 * @param schema_ptr Pointer to Arrow schema (FFI_ArrowSchema)
 * @return 0 on success, negative error code on failure
 */
int32_t write_batch(int64_t dataset_handle, struct ArrowArray* array_ptr, struct ArrowSchema* schema_ptr);

/**
 * Close a dataset and release resources
 * 
 * @param dataset_handle Handle returned by create_or_open_dataset
 * @return 0 on success, negative error code on failure
 */
int32_t close_dataset(int64_t dataset_handle);

/**
 * Get the last error message
 * 
 * @return Pointer to null-terminated error string, or null if no error
 */
const char* lance_get_last_error(void);

#ifdef __cplusplus
}
#endif

#endif // LANCE_BRIDGE_H

