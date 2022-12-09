#include "LUT.h"

node *LUT::makeEmpty(node *t) {
        if (t == NULL)
                return NULL;
        {
                makeEmpty(t->left);
                makeEmpty(t->right);
                delete t;
        }
        return NULL;
}

int LUT::find(node *t, int128_t search_index) {
        if (t == NULL)
                return -1;
        else if (search_index < t->search_index)
                return find(t->left, search_index);
        else if (search_index > t->search_index)
                return find(t->right, search_index);
        else
                return t->arm_index;
}

node *LUT::insert(int arm_index, int128_t search_index, node *t) {
        if (t == NULL) {
                t = new node;
                t->search_index = search_index;
                t->arm_index = arm_index;
                t->left = t->right = NULL;
        } else if (search_index < t->search_index)
                t->left = insert(arm_index, search_index, t->left);
        else if (search_index > t->search_index)
                t->right = insert(arm_index, search_index, t->right);
        return t;
}

node *LUT::clone(node *ptr) {
        if (ptr) {
                //node *newnode = new node(ptr->data);
                //newnode->left = clone(ptr->left);
                //newnode->right = clone(ptr->right);
                //return newnode;
                //std::cout << "22" <<std::endl;
        }
        return NULL;
        //std::cout << "33" <<std::endl;
}

LUT::LUT() {
        root = NULL;
        //std::cout << "create"<< std::endl;
}

// PSO
LUT::LUT(const LUT &old_LUT) {
        //std::cout << "copy" << std::endl;
        //*root = *old_LUT.root;

        root = clone(old_LUT.root);
}
// PSO

LUT::~LUT() {
        root = makeEmpty(root);
        //std::cout << "delete"<< std::endl;
}

/*
BST<T>::Node * BST<T>::clone(Node * ptr) const
{
  if (ptr)
  {
    Node *newnode = new Node(ptr->data);
    newnode->left = clone(ptr->left);
    newnode->right = clone(ptr->right);
    return newnode;
  }
  return NULL;
}*/
