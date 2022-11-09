#ifndef _LUT_H_
#define _LUT_H_
#include <iostream>
#include <boost/multiprecision/cpp_int.hpp>
using namespace boost::multiprecision;

struct node {
        int128_t search_index;
        int arm_index;
        node* left;
        node* right;
    };

class LUT
{
    
    
    
    node* root;
    
    node* makeEmpty(node* t);
    
    
public:
    
    void insert(int arm_index, int128_t search_index) {
        root = insert(arm_index, search_index, root);
    }

    int search(int128_t search_index) {
        return find(root, search_index);
    }
    
    int find(node* t, int128_t search_index);
    node* insert(int arm_index, int128_t search_index, node* t);
    node* clone(node * ptr);
    LUT();
    LUT(const LUT& old_LUT); // PSO COPY
    ~LUT();

};

#endif // _LUT_H_


