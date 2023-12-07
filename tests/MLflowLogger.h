#ifndef MEMO_MLFLOWLOGGER_H
#define MEMO_MLFLOWLOGGER_H

#include <string>

class MLflowLogger {
       public:
        std::string get_or_create_experiment(const std::string &experiment_name);
        std::string start_run(const std::string &experiment_id);
        void log_param(const std::string &run_id, const std::string &key, const std::string &value);
        void log_metric(const std::string &run_id, const std::string &key, double value);
        void upload_artifact(const std::string &run_id, const std::string &file_path);
        MLflowLogger();

       private:
        std::string mlflow_url;
        std::string mlflow_access_token;
        static const std::string GET_EXPERIMENT_BY_NAME_ENDPOINT;
        static const std::string CREATE_EXPERIMENT_ENDPOINT;
        static const std::string CREATE_RUN_ENDPOINT;
        static const std::string LOG_PARAMETER_ENDPOINT;
        static const std::string LOG_METRIC_ENDPOINT;
        static const std::string GET_RUN_INFO_ENDPOINT;
};

#endif//MEMO_MLFLOWLOGGER_H
