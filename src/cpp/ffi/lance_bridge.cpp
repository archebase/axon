#include "lance_bridge.h"
#include <arrow/c/bridge.h>
#include <arrow/api.h>
#include <memory>
#include <stdexcept>
#include <iostream>

extern "C" {
    // These are the actual Rust functions
    int64_t create_or_open_dataset(const char* path, struct ArrowSchema* schema_ptr);
    int32_t write_batch(int64_t dataset_handle, struct ArrowArray* array_ptr, struct ArrowSchema* schema_ptr);
    int32_t close_dataset(int64_t dataset_handle);
    const char* lance_get_last_error(void);
}

namespace lance_bridge {

class DatasetHandle {
public:
    explicit DatasetHandle(int64_t handle) : handle_(handle) {}
    
    int64_t get() const { return handle_; }
    
    bool valid() const { return handle_ > 0; }
    
private:
    int64_t handle_;
};

// C++ wrapper class for easier use
class LanceWriter {
public:
    LanceWriter() : handle_(0) {}
    
    ~LanceWriter() {
        if (handle_.valid()) {
            close();
        }
    }
    
    // Disable copy constructor and assignment
    LanceWriter(const LanceWriter&) = delete;
    LanceWriter& operator=(const LanceWriter&) = delete;
    
    // Enable move constructor and assignment
    LanceWriter(LanceWriter&& other) noexcept : handle_(other.handle_) {
        other.handle_ = DatasetHandle(0);
    }
    
    LanceWriter& operator=(LanceWriter&& other) noexcept {
        if (this != &other) {
            if (handle_.valid()) {
                close();
            }
            handle_ = other.handle_;
            other.handle_ = DatasetHandle(0);
        }
        return *this;
    }
    
    bool open(const std::string& path, const std::shared_ptr<arrow::Schema>& schema) {
        if (path.empty()) {
            std::cerr << "LanceWriter: Invalid path (empty)" << std::endl;
            return false;
        }
        
        if (!schema) {
            std::cerr << "LanceWriter: Invalid schema (null)" << std::endl;
            return false;
        }
        
        if (handle_.valid()) {
            close();
        }
        
        struct ArrowSchema c_schema;
        arrow::Status status = arrow::ExportSchema(*schema, &c_schema);
        if (!status.ok()) {
            std::cerr << "LanceWriter: Failed to export schema: " << status.ToString() << std::endl;
            return false;
        }
        
        int64_t h = create_or_open_dataset(path.c_str(), &c_schema);
        
        // Release schema
        c_schema.release(&c_schema);
        
        if (h > 0) {
            handle_ = DatasetHandle(h);
            return true;
        } else {
            const char* error = lance_get_last_error();
            if (error) {
                std::cerr << "LanceWriter: Failed to open dataset: " << error << std::endl;
            }
            return false;
        }
    }
    
    bool write(const std::shared_ptr<arrow::RecordBatch>& batch) {
        if (!handle_.valid()) {
            std::cerr << "LanceWriter: Invalid dataset handle" << std::endl;
            return false;
        }
        
        if (!batch) {
            std::cerr << "LanceWriter: Invalid batch (null)" << std::endl;
            return false;
        }
        
        struct ArrowArray c_array;
        struct ArrowSchema c_schema;
        
        arrow::Status status = arrow::ExportRecordBatch(*batch, &c_array, &c_schema);
        if (!status.ok()) {
            std::cerr << "LanceWriter: Failed to export batch: " << status.ToString() << std::endl;
            return false;
        }
        
        int32_t result = write_batch(handle_.get(), &c_array, &c_schema);
        
        // Release C structures
        c_array.release(&c_array);
        c_schema.release(&c_schema);
        
        if (result != LANCE_SUCCESS) {
            const char* error = lance_get_last_error();
            if (error) {
                std::cerr << "LanceWriter: Write failed: " << error << std::endl;
            }
            return false;
        }
        
        return true;
    }
    
    void close() {
        if (handle_.valid()) {
            int32_t result = close_dataset(handle_.get());
            if (result != LANCE_SUCCESS) {
                const char* error = lance_get_last_error();
                if (error) {
                    std::cerr << "LanceWriter: Close failed: " << error << std::endl;
                }
            }
            handle_ = DatasetHandle(0);
        }
    }
    
    std::string getLastError() const {
        const char* err = lance_get_last_error();
        return err ? std::string(err) : std::string();
    }
    
    bool isOpen() const {
        return handle_.valid();
    }
    
private:
    DatasetHandle handle_;
};

} // namespace lance_bridge
