#include "MLflowLogger.h"
#include "SecretManagement.h"
#include <cpr/cpr.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

const std::string MLflowLogger::GET_EXPERIMENT_BY_NAME_ENDPOINT = "/api/2.0/mlflow/experiments/get-by-name";
const std::string MLflowLogger::CREATE_EXPERIMENT_ENDPOINT = "/api/2.0/mlflow/experiments/create";
const std::string MLflowLogger::CREATE_RUN_ENDPOINT = "/api/2.0/mlflow/runs/create";
const std::string MLflowLogger::LOG_PARAMETER_ENDPOINT = "/api/2.0/mlflow/runs/log-parameter";
const std::string MLflowLogger::LOG_METRIC_ENDPOINT = "/api/2.0/mlflow/runs/log-metric";
const std::string MLflowLogger::GET_RUN_INFO_ENDPOINT = "/api/2.0/mlflow/runs/get";

MLflowLogger::MLflowLogger() {
        try {
                mlflow_access_token = SecretManagement::get_secret("TeamMoos", "MLflow", "password");
                mlflow_url = SecretManagement::get_secret("TeamMoos", "MLflow", "username");

                // Use the mlflow_access_token in your application
        } catch (const std::exception &e) {
                std::cerr << "Error retrieving secret: " << e.what() << std::endl;
                // Handle errors appropriately
        }
}

auto parse_experiment_id(const std::string &response) -> std::string {
        // Parse the JSON response
        auto json = nlohmann::json::parse(response);
        // Extract the experiment_id
        if (json.contains("experiment_id")) {
                std::string experiment_id = json["experiment_id"].get<std::string>();
                return experiment_id;
        }
        if (json.contains("experiment")) {
                std::string experiment_id = json["experiment"]["experiment_id"].get<std::string>();
                return experiment_id;
        }
        std::cerr << "Failed to parse experiment ID: " << response << std::endl;
        exit(1);
}

auto read_file_to_string(const std::string &file_path) -> std::string {
        if (!std::filesystem::exists(file_path)) {
                throw std::runtime_error("File does not exist: " + file_path);
        }

        std::ifstream file(file_path, std::ios::binary);
        if (!file.is_open()) {
                throw std::runtime_error("Could not open file: " + file_path);
        }

        return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

auto get_artifact_uri(const std::string &response) -> std::string {
        auto json = nlohmann::json::parse(response);
        std::string artifact_uri = json["run"]["info"]["artifact_uri"];
        return artifact_uri;
}

auto MLflowLogger::get_or_create_experiment(const std::string &experiment_name) -> std::string {
        std::string url = MLflowLogger::mlflow_url + MLflowLogger::GET_EXPERIMENT_BY_NAME_ENDPOINT;
        // First, try to get the experiment ID by name
        cpr::Response r{};
        r = cpr::Get(cpr::Url{url}, cpr::Parameters{{"experiment_name", experiment_name}}, cpr::Header{{"Authorization", "Bearer " + MLflowLogger::mlflow_access_token}});

        if (r.status_code == 200) {
                // Parse the response to get the experiment ID
                std::string experiment_id = parse_experiment_id(r.text);
                return experiment_id;
        }

        // If the experiment does not exist, create a new one
        r = cpr::Post(cpr::Url{MLflowLogger::mlflow_url + MLflowLogger::CREATE_EXPERIMENT_ENDPOINT}, cpr::Payload{{"name", experiment_name}}, cpr::Header{{"Authorization", "Bearer " + MLflowLogger::mlflow_access_token}});

        if (r.status_code == 200) {
                // Extract the experiment ID from the creation response
                std::string experiment_id = parse_experiment_id(r.text);
                return experiment_id;
        }

        std::cerr << "Failed to get or create experiment: " << r.text << std::endl;
        return "";// Return empty string in case of failure
}

auto parse_run_id(const std::string &response) -> std::string {
        // Parse the JSON response
        auto json = nlohmann::json::parse(response);
        // Extract the run_id
        std::string run_id = json["run"]["info"]["run_id"];
        return run_id;
}

auto MLflowLogger::start_run(const std::string &experiment_id) -> std::string {
        std::string url = MLflowLogger::mlflow_url + MLflowLogger::CREATE_RUN_ENDPOINT;
        cpr::Response r{};
        r = cpr::Post(cpr::Url{url}, cpr::Payload{{"experiment_id", experiment_id}}, cpr::Header{{"Authorization", "Bearer " + MLflowLogger::mlflow_access_token}});

        if (r.status_code == 200) {
                // Parse the response to get the run ID
                std::string run_id = parse_run_id(r.text);
                return run_id;
        }

        std::cerr << "Failed to start a new run: " << r.text << std::endl;
        return "";// Return empty string in case of failure
}

void MLflowLogger::log_param(const std::string &run_id, const std::string &key, const std::string &value) {
        std::string url = MLflowLogger::mlflow_url + MLflowLogger::LOG_PARAMETER_ENDPOINT;
        cpr::Response r{};
        r = cpr::Post(cpr::Url{url}, cpr::Payload{{"run_id", run_id}, {"key", key}, {"value", value}}, cpr::Header{{"Authorization", "Bearer " + MLflowLogger::mlflow_access_token}});
        if (r.status_code != 200) {
                std::cerr << "Failed to log param to MLflow: " << r.text << std::endl;
        }
}

void MLflowLogger::log_metric(const std::string &run_id, const std::string &key, double value) {
        std::string url = MLflowLogger::mlflow_url + MLflowLogger::LOG_METRIC_ENDPOINT;
        cpr::Response r{};
        r = cpr::Post(cpr::Url{url}, cpr::Payload{{"run_id", run_id}, {"key", key}, {"value", std::to_string(value)}, {"timestamp", std::to_string(time(nullptr))}}, cpr::Header{{"Authorization", "Bearer " + MLflowLogger::mlflow_access_token}});
        if (r.status_code != 200) {
                std::cerr << "Failed to log metric to MLflow: " << r.text << std::endl;
        }
}

void MLflowLogger::upload_artifact(const std::string &run_id, const std::string &file_path) {
        std::string url = MLflowLogger::mlflow_url + MLflowLogger::GET_RUN_INFO_ENDPOINT;
        // Send the POST request with the file content
        cpr::Response r{};
        r = cpr::Get(cpr::Url{url}, cpr::Parameters{{"run_id", run_id}}, cpr::Header{{"Authorization", "Bearer " + MLflowLogger::mlflow_access_token}});

        if (r.status_code != 200) {
                std::cerr << "Failed to get run info: " << r.text << std::endl;
                return;
        }

        // Extract the artifact_uri from the response JSON
        std::string artifact_uri = get_artifact_uri(r.text);

        // Read the file content into a string
        std::string file_content;
        try {
                file_content = read_file_to_string(file_path);
        } catch (const std::exception &e) {
                std::cerr << "Error reading file: " << e.what() << std::endl;
                return;
        }

        // Send the PUT request with the file content
        r = cpr::Put(cpr::Url{artifact_uri + file_path}, cpr::Body{file_content}, cpr::Header{{"Authorization", "Bearer " + MLflowLogger::mlflow_access_token}, {"Content-Type", "text/plain"}});

        if (r.status_code != 201) {
                std::cerr << "Failed to upload artifact to MLflow: " << r.text << std::endl;
        }
}
