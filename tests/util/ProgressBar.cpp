#include "ProgressBar.h"

ProgressBar::ProgressBar(int total, int width) : total_(total), width_(width) {}

void ProgressBar::update(int new_progress) const {
        int progress = (int) (static_cast<double>(new_progress) / total_ * 100);

        std::cout << "\rConfigs: [";
        int progressPosition = width_ * progress / 100;
        for (int barPosition = 0; barPosition < width_; ++barPosition) {
                if (barPosition < progressPosition)
                        std::cout << "=";
                else if (barPosition == progressPosition)
                        std::cout << ">";
                else
                        std::cout << " ";
        }
        std::cout << "] " << new_progress << "/" << total_ << " | " << progress << "%" << std::flush;
}

void ProgressBar::finish() const {
        update(total_);
        std::cout << "\n\nAll Configs done!" << std::endl;
}

ProgressBar::ProgressBar() {
        total_ = 0;
        width_ = 0;
}
