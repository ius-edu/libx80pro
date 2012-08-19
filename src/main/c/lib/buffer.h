typedef struct Buffer
{
    byte* data;
    int length;

    int *(size)(Buffer*);
} Buffer;

Buffer* new_Buffer(int);
void delete_Buffer(Buffer*);
