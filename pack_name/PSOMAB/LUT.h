#ifndef _LUT_H_
#define _LUT_H_
#include <boost/multiprecision/cpp_int.hpp>
#include <iostream>
using namespace boost::multiprecision;

// ToDo: schauen ob HashTable besser wäre
// ToDo: überprüfen ob es möglich ist diesen großen Int loszuwerden

struct node {
        int128_t search_index;
        int arm_index;
        node *left;
        node *right;
};

class LUT {

        node *root;

        node *makeEmpty(node *t);

       public:
        void insert(int arm_index, int128_t search_index) {
                root = insert(arm_index, search_index, root);
        }

        int search(int128_t search_index) {
                return find(root, search_index);
        }

        int find(node *t, int128_t search_index);
        node *insert(int arm_index, int128_t search_index, node *t);
        node *clone(node *ptr);
        LUT();
        LUT(const LUT &old_LUT);// PSO COPY
        ~LUT();
};

#endif// _LUT_H_
