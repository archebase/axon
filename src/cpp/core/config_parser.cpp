#include "config_parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

namespace axon {
namespace core {

RecorderConfig RecorderConfig::from_yaml(const std::string& yaml_path) {
    ConfigParser parser;
    RecorderConfig config;
    if (!parser.load_from_file(yaml_path, config)) {
        std::cerr << "Failed to load config from: " << yaml_path << std::endl;
    }
    return config;
}

RecorderConfig RecorderConfig::from_yaml_string(const std::string& yaml_content) {
    ConfigParser parser;
    RecorderConfig config;
    if (!parser.load_from_string(yaml_content, config)) {
        std::cerr << "Failed to load config from string" << std::endl;
    }
    return config;
}

bool RecorderConfig::validate() const {
    if (dataset.path.empty()) {
        return false;
    }
    
    if (topics.empty()) {
        return false;
    }
    
    for (const auto& topic : topics) {
        if (topic.name.empty() || topic.message_type.empty()) {
            return false;
        }
        if (topic.batch_size == 0) {
            return false;
        }
    }
    
    if (dataset.mode != "create" && dataset.mode != "append") {
        return false;
    }
    
    return true;
}

std::string RecorderConfig::to_string() const {
    std::ostringstream oss;
    oss << "Dataset: " << dataset.path << " (mode: " << dataset.mode << ")\n";
    oss << "Topics:\n";
    for (const auto& topic : topics) {
        oss << "  - " << topic.name << " (" << topic.message_type 
            << ", batch_size: " << topic.batch_size 
            << ", flush_interval: " << topic.flush_interval_ms << "ms)\n";
    }
    oss << "Recording: auto_start=" << recording.auto_start 
        << ", max_disk_usage=" << recording.max_disk_usage_gb << "GB\n";
    return oss.str();
}

bool ConfigParser::load_from_file(const std::string& path, RecorderConfig& config) {
    try {
        YAML::Node node = YAML::LoadFile(path);
        return load_from_string(YAML::Dump(node), config);
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
        return false;
    }
}

bool ConfigParser::load_from_string(const std::string& yaml_content, RecorderConfig& config) {
    try {
        YAML::Node node = YAML::Load(yaml_content);
        
        // Parse dataset config
        if (node["dataset"]) {
            parse_dataset(node["dataset"], config.dataset);
        }
        
        // Parse topics
        if (node["topics"]) {
            parse_topics(node["topics"], config.topics);
        }
        
        // Parse recording config
        if (node["recording"]) {
            parse_recording(node["recording"], config.recording);
        }
        
        return config.validate();
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
        return false;
    }
}

bool ConfigParser::save_to_file(const std::string& path, const RecorderConfig& config) {
    try {
        YAML::Node node;
        
        // Dataset
        node["dataset"]["path"] = config.dataset.path;
        node["dataset"]["mode"] = config.dataset.mode;
        
        // Topics
        node["topics"] = YAML::Node(YAML::NodeType::Sequence);
        for (const auto& topic : config.topics) {
            YAML::Node topic_node;
            topic_node["name"] = topic.name;
            topic_node["message_type"] = topic.message_type;
            topic_node["batch_size"] = topic.batch_size;
            topic_node["flush_interval_ms"] = topic.flush_interval_ms;
            node["topics"].push_back(topic_node);
        }
        
        // Recording
        node["recording"]["auto_start"] = config.recording.auto_start;
        node["recording"]["max_disk_usage_gb"] = config.recording.max_disk_usage_gb;
        
        std::ofstream file(path);
        file << node;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to save config: " << e.what() << std::endl;
        return false;
    }
}

bool ConfigParser::parse_dataset(const YAML::Node& node, DatasetConfig& dataset) {
    if (node["path"]) {
        dataset.path = node["path"].as<std::string>();
    }
    if (node["mode"]) {
        dataset.mode = node["mode"].as<std::string>();
    }
    return true;
}

bool ConfigParser::parse_topics(const YAML::Node& node, std::vector<TopicConfig>& topics) {
    if (!node.IsSequence()) {
        return false;
    }
    
    topics.clear();
    for (const auto& topic_node : node) {
        TopicConfig topic;
        if (topic_node["name"]) {
            topic.name = topic_node["name"].as<std::string>();
        }
        if (topic_node["message_type"]) {
            topic.message_type = topic_node["message_type"].as<std::string>();
        }
        if (topic_node["batch_size"]) {
            topic.batch_size = topic_node["batch_size"].as<size_t>();
        }
        if (topic_node["flush_interval_ms"]) {
            topic.flush_interval_ms = topic_node["flush_interval_ms"].as<int>();
        }
        topics.push_back(topic);
    }
    return true;
}

bool ConfigParser::parse_recording(const YAML::Node& node, RecordingConfig& recording) {
    if (node["auto_start"]) {
        recording.auto_start = node["auto_start"].as<bool>();
    }
    if (node["max_disk_usage_gb"]) {
        recording.max_disk_usage_gb = node["max_disk_usage_gb"].as<double>();
    }
    return true;
}

bool ConfigParser::validate(const RecorderConfig& config, std::string& error_msg) {
    if (!config.validate()) {
        error_msg = "Configuration validation failed";
        return false;
    }
    return true;
}

} // namespace core
} // namespace axon

