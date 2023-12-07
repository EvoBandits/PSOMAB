#ifndef MEMO_SECRETMANAGEMENT_H
#define MEMO_SECRETMANAGEMENT_H

#include <string>
#include <unordered_map>

class SecretManagement {
       public:
        static std::string get_secret_from_1password(const std::string &item_name, const std::string &field_name);
        static std::unordered_map<std::string, std::string> read_dotenv(const std::string &filename);
        static std::string get_secret(const std::string &item_name, const std::string &field_name);
};

#endif//MEMO_SECRETMANAGEMENT_H
