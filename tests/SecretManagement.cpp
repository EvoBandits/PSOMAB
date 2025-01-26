#include "SecretManagement.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>

#ifndef _WIN32
auto SecretManagement::get_secret_from_1password(const std::string &vault_name, const std::string &item_name, const std::string &field_name) -> std::string {
        std::string command = "op read op://" + vault_name + "/" + item_name + "/" + field_name;

        std::array<char, 128> buffer{};
        std::string result;
        FILE *pipe = popen(command.c_str(), "r");

        if (!pipe) {
                throw std::runtime_error("Failed to execute 1Password command.");
        }

        try {
                while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
                        result += buffer.data();
                }
        } catch (...) {
                pclose(pipe);// Ensure pipe is closed in case of an exception
                throw;       // Re-throw the caught exception
        }

        int returnCode = pclose(pipe);
        if (returnCode != 0) {
                if (returnCode == 32512) {// Command not found
                        throw std::runtime_error("1Password command not found");
                } else {
                        throw std::runtime_error("1Password command failed with code: " + std::to_string(returnCode));
                }
        }

        return result;
}
#endif

auto SecretManagement::read_dotenv(const std::string &filename) -> std::unordered_map<std::string, std::string> {
        std::unordered_map<std::string, std::string> env;
        std::ifstream file(filename);

        // Check for file open failure
        if (!file) {
                throw std::runtime_error("Failed to open file: " + filename);
        }

        std::string line;
        while (std::getline(file, line)) {
                size_t delimiterPos = line.find('=');
                if (delimiterPos != std::string::npos) {// Ensure delimiter is found
                        std::string key = line.substr(0, delimiterPos);
                        std::string value = line.substr(delimiterPos + 1);
                        env[std::move(key)] = std::move(value);// Use std::move for efficiency
                }
        }

        return env;
}

auto SecretManagement::get_secret(const std::string &vault_name, const std::string &item_name, const std::string &field_name) -> std::string {
#ifndef _WIN32
        try {
                std::string secret = get_secret_from_1password(vault_name, item_name, field_name);
                if (!secret.empty()) {
                        secret.erase(std::remove(secret.begin(), secret.end(), '\n'), secret.end());
                        return secret;
                }
        } catch (const std::runtime_error &e) {
                if (std::string(e.what()) == "1Password command not found") {
                        std::cerr << "1Password not found, falling back to .env" << std::endl;
                } else {
                        std::cerr << "Error: " << e.what() << std::endl;
                }
        }
#endif

        auto env_vars = read_dotenv(".env");
        auto it = env_vars.find(field_name);
        if (it != env_vars.end()) {
                return it->second;
        }

        throw std::runtime_error("Secret not found: " + field_name);
}
