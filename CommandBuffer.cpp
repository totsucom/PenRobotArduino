#include <Arduino.h>
#include "CommandBuffer.h"

CommandBuffer::CommandBuffer(int buffer_size)
{
    pBuf = new unsigned char [buffer_size];
    this->buffer_size = buffer_size;
    write_index = 0;
    read_index = 0;
    num_command = 0;
}

CommandBuffer::~CommandBuffer()
{
    delete []pBuf;
}

int CommandBuffer::getFreeSize()
{
    return buffer_size - write_index + read_index;
}

int CommandBuffer::getCount()
{
    return num_command;
}

void CommandBuffer::_addUChar(unsigned char c)
{
    *(pBuf + write_index) = c;
    if (++write_index >= buffer_size) write_index -= buffer_size;
}

void CommandBuffer::_readUChar(unsigned char *c)
{
    *c = *(pBuf + read_index);
    if (++read_index >= buffer_size) read_index -= buffer_size;
}

void CommandBuffer::_addFloat(float f)
{
    unsigned char *p = (unsigned char *)&f;
    for (int i = 0; i < 4; i++) {
        *(pBuf + write_index) = *p++;
        if (++write_index >= buffer_size) write_index -= buffer_size;
    }
}

void CommandBuffer::_readFloat(float *f)
{
    unsigned char *p = (unsigned char *)f;
    for (int i = 0; i < 4; i++) {
        *p++ = *(pBuf + read_index);
        if (++read_index >= buffer_size) read_index -= buffer_size;
    }
}

void CommandBuffer::_addInt(int d)
{
    unsigned char *p = (unsigned char *)&d;
    for (int i = 0; i < 4; i++) {
        *(pBuf + write_index) = *p++;
        if (++write_index >= buffer_size) write_index -= buffer_size;
    }
}

void CommandBuffer::_readInt(int *d)
{
    unsigned char *p = (unsigned char *)d;
    for (int i = 0; i < 4; i++) {
        *p++ = *(pBuf + read_index);
        if (++read_index >= buffer_size) read_index -= buffer_size;
    }
}

bool CommandBuffer::addCommand(unsigned char *p, int length)
{
    if (getFreeSize() < length) return false;
    while (length-- > 0) {
        _addUChar(*p++);
    }
    num_command++;
    return true;
}

bool CommandBuffer::addCommand(unsigned char cmd)
{
    if (getFreeSize() < 1) return false;
    _addUChar(cmd);
    num_command++;
    return true;
}

bool CommandBuffer::readCommand(unsigned char *cmd)
{
    if (num_command == 0) return false;
    _readUChar(cmd);
    num_command--;
    return true;
}

bool CommandBuffer::addCommand(unsigned char cmd, float a)
{
    if (getFreeSize() < 5) return false;
    _addUChar(cmd);
    _addFloat(a);
    num_command++;
    return true;
}

bool CommandBuffer::readCommand(unsigned char *cmd, float *a)
{
    if (num_command == 0) return false;
    _readUChar(cmd);
    _readFloat(a);
    num_command--;
    return true;
}

bool CommandBuffer::addCommand(unsigned char cmd, float a, float b)
{
    if (getFreeSize() < 9) return false;
    _addUChar(cmd);
    _addFloat(a);
    _addFloat(b);
    num_command++;
    return true;
}

bool CommandBuffer::readCommand(unsigned char *cmd, float *a, float *b)
{
    if (num_command == 0) return false;
    _readUChar(cmd);
    _readFloat(a);
    _readFloat(b);
    num_command--;
    return true;
}

bool CommandBuffer::addCommand(unsigned char cmd, int a)
{
    if (getFreeSize() < 5) return false;
    _addUChar(cmd);
    _addInt(a);
    num_command++;
    return true;
}

bool CommandBuffer::readCommand(unsigned char *cmd, int *a)
{
    if (num_command == 0) return false;
    _readUChar(cmd);
    _readInt(a);
    num_command--;
    return true;
}

//Error -1
int CommandBuffer::peekCommand()
{
    if (num_command == 0) return -1;
    return *(pBuf + read_index);
}
