#ifndef hwa_base_coder_buffer_h
#define hwa_base_coder_buffer_h

#include <string>
#include <deque>
#include <queue>

namespace hwa_base
{
    /**
    *@brief Class for base_coder_buffer
    */
    class base_coder_buffer
    {
    public:
        /** @brief default constructor. */
        base_coder_buffer();

        /** @brief default destructor. */
        ~base_coder_buffer();

        /** @brief get size. */
        int size();

        /** @brief add. */
        int add(char *buff, int size);

        /**
        * @brief get single line from the buffer.
        * @param[in]  str        the content of the single line
        * @param[in]  from_pos    the position in the buffer
        * @return      int
        */
        int getline(std::string &str, int from_pos);

        /** @brief remove from buffer. */
        int consume(int bytes_to_eat);

        /** @brief tostd::string. */
        void toString(std::string &str);

    private:
        std::deque<char> _buffer; ///< buffer
    };

    /**
    *@brief Class for base_coder_char_buffer
    */
    class base_coder_char_buffer
    {
    public:
        /** @brief default constructor. */
        base_coder_char_buffer();

        /** @brief default destructor. */
        ~base_coder_char_buffer();

        /** @brief get size. */
        int size();

        /** @brief add. */
        int add(char *buff, int size);

        /**
        * @brief get single line from the buffer.
        * @param[in]  str        the content of the single line
        * @param[in]  from_pos    the position in the buffer
        * @return      int
        */
        int getline(std::string &str, int from_pos);

        /**
        * @brief get the buffer.
        * @param[in]  buff        buffer
        * @return      int
        */
        int getbuffer(const char *&buff);

        /** @brief remove from buffer. */
        int consume(int bytes_to_eat);

        int ex_consume(int bytes_to_eat);

        /** @brief to std::string. */
        void toString(std::string &str);

    private:
        int _begpos;   ///< begin position in the buffer
        int _endpos;   ///< end position in the buffer
        int _buffsz;   ///< buffer size
        char *_buffer; ///< buffer
    };
}

#endif