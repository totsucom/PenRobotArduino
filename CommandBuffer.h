#ifndef COMMANDBUFFER_H_INCLUDE
#define COMMANDBUFFER_H_INCLUDE

/* シリアルポートから入ってくるコマンドをキューで管理するクラス */

class CommandBuffer {
    private:
        unsigned char *pBuf;
        int buffer_size;
        int write_index;
        int read_index;
        int num_command;

        void _addUChar(unsigned char c);
        void _readUChar(unsigned char *c);
        void _addFloat(float f);
        void _readFloat(float *f);
        void _addInt(int d);
        void _readInt(int *d);
        
    public:
        CommandBuffer(int buffer_size);
        ~CommandBuffer();
        int getFreeSize();
        int getCount();
        bool addCommand(unsigned char *p, int length);
        bool addCommand(unsigned char cmd);
        bool readCommand(unsigned char *cmd);
        bool addCommand(unsigned char cmd, float a);
        bool readCommand(unsigned char *cmd, float *a);
        bool addCommand(unsigned char cmd, float a, float b);
        bool readCommand(unsigned char *cmd, float *a, float *b);
        bool addCommand(unsigned char cmd, int a);
        bool readCommand(unsigned char *cmd, int *a);
        int peekCommand();
};

#endif
