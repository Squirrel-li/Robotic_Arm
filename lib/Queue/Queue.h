#ifndef QUEUE_H
#define QUEUE_H

#include <Arduino.h>

template <typename T, size_t N>
class Queue {
public:
    Queue(bool isReplaceable = false);
    bool push(const T& item);
    bool pushOverwrite(const T& item, T* overwritten = nullptr); // 新增
    bool pop(T* item);
    bool isFull() const;
    bool isEmpty() const;
    size_t size() const { return _size; }

private:
    T _items[N];
    size_t _head;
    size_t _tail;
    size_t _size;
    bool _isReplaceable;

    static inline size_t nextIndex(size_t idx) {
        idx++;
        return (idx >= N) ? 0 : idx;
    }
};

template <typename T, size_t N>
Queue<T, N>::Queue(bool isReplaceable)
    : _head(0), _tail(0), _size(0), _isReplaceable(isReplaceable) {}

template <typename T, size_t N>
bool Queue<T, N>::push(const T& item) {
    if (_size == N) {
        if (!_isReplaceable) return false;
        _head = nextIndex(_head);
        _size--;
    }

    _items[_tail] = item;
    _tail = nextIndex(_tail);
    _size++;
    return true;
}

template <typename T, size_t N>
bool Queue<T, N>::pushOverwrite(const T& item, T* overwritten) {
    if (_size == N) {
        if (!_isReplaceable) return false;
        if (overwritten) *overwritten = _items[_head];
        _items[_tail] = item;
        _head = nextIndex(_head);
        _tail = nextIndex(_tail);
        return true;
    }

    _items[_tail] = item;
    _tail = nextIndex(_tail);
    _size++;
    return true;
}

template <typename T, size_t N>
bool Queue<T, N>::pop(T* item) {
    if (_size == 0) return false;
    if (item) *item = _items[_head];
    _head = nextIndex(_head);
    _size--;
    return true;
}

template <typename T, size_t N>
bool Queue<T, N>::isFull() const {
    return _size == N;
}

template <typename T, size_t N>
bool Queue<T, N>::isEmpty() const {
    return _size == 0;
}

#endif // QUEUE_H