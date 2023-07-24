/*
 * Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <deque>

namespace gui_node
{

/**
 * The fixed size deque.
 *
 * @tparam T Data type to hold.
 */
template <typename T>
class FixedDeque : public std::deque<T>
{
private:
    size_t max_size; ///< Maximum size of the deque
public:
    /**
     * Constructor.
     *
     * @param size The size of the deque.
     */
    FixedDeque(size_t size) : max_size(size) {}

    /**
     * Pushes the value to the front of the deque.
     * Removes the last element if the deque is full.
     *
     * @param val The value to push to the front of the deque.
     */
    void push_front(const T &val)
    {
        if (std::deque<T>::size() >= max_size)
        {
            std::deque<T>::pop_back();
        }
        std::deque<T>::push_front(val);
    }
};

} // namespace gui_node
