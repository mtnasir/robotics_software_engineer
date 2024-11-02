#include <iostream>
using namespace std;

class Node
{
public:
    int value;
    Node *next;
    Node(int value)
    {
        this->value = value;
        next = nullptr;
    }
};

class LinkedList
{

public:
    int length;
    Node *head;
    Node *tail;

    LinkedList(int value)
    {
        Node *newNode = new Node(value);
        head = newNode;
        tail = newNode;
        length = 1;
    }
    void PrintList()
    {
        Node *temp = head;
        while (temp != nullptr)
        {
            cout << temp->value << endl;
            temp = temp->next;
        }
    }
    void getHead()
    {
        cout << "Head: " << head->value << endl;
    }
    void getTail()
    {
        cout << "tail :" << tail->value << endl;
    }
    void getLength()
    {
        cout << "Tail: " << length << endl;
    }
    void append(int value)
    {

        Node *newNode = new Node(value);
       if (length==0){
            head=newNode;
            tail=newNode;
            }else{
                tail->next=newNode;
                tail=newNode;
           }
            length++;
    }
    void deleteLast()
    {
        if(length==0) return;
        Node* temp=head;
        if (length==1){
            head=nullptr;
            tail=nullptr;
        }else{
            Node* pre=head;
            while(temp->next){
                pre=temp;
                temp=temp->next;
            }
            tail=pre;
            tail->next=nullptr;
        }
    }
    void prepend(int value){
        Node* newNode= new Node(value);
        if (length==0){
            head=newNode;
            tail=newNode;
        }else{
            newNode->next=head;
            head=newNode;
        }
        length++;
    }
    void deleteFirst(){
        Node* temp= head;
        if (length==0) {
            return;
        }
        if (length==1){
            head=nullptr;
            tail=nullptr;
        }
        else{
            head=head->next;
        }
        delete(temp);
        length--;

    }
    ~LinkedList()
    {
        Node *temp = head;
        while (head)
        {
            head = head->next;
            cout << "deleting:  " << temp->value << endl;

            delete (temp);
            temp = head;
        }
    }
};

int main(int argc, char **argv)
{

    LinkedList *N1 = new LinkedList(5);
    N1->prepend(3);
    N1->prepend(13);
    N1->PrintList();
    N1->deleteLast();
    N1->PrintList();
    N1->deleteFirst();
    N1->PrintList();
    
    cout << "done" << endl;
    delete (N1);
}