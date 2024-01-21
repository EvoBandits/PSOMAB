
#ifndef MEMO_TESTS_UTIL_PROGRESSBAR_H_
#define MEMO_TESTS_UTIL_PROGRESSBAR_H_

#include <iostream>

class ProgressBar {
       public:
        ProgressBar();
        explicit ProgressBar(int total, int width = 40);
        void update(int new_progress) const;
        void finish() const;

       private:
        int total_;
        int width_;
};

#endif//MEMO_TESTS_UTIL_PROGRESSBAR_H_
