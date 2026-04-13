#ifndef hwa_base_note_h
#define hwa_base_note_h

#include <map>
#include <set>
#include <string>
#include <vector>
#include <memory>
#include <stdio.h>
#include <fstream>
#include "hwa_base_mutex.h"

#define BUF_SIZE 1024

namespace hwa_base
{

    /** @brief the type of NOTE. */
    enum base_note_type
    {
        GERROR,
        GWARNING,
        GMESSAGE
    };

    /** @brief class for base_note. */
    class base_note
    {

    public:
        /** @brief constructor 1. */
        base_note(base_note_type n, std::string f, std::string s);

        /** @brief default destructor. */
        virtual ~base_note();

        /** @brief util function. */
        std::string str() const { return _str() + _text; }

        std::string note() const { return _str(); }
        std::string text() const { return _text; }
        std::string func() const { return _func; }
        base_note_type status() const { return _stat; }

        /** @brief override operator. */
        bool operator<(const base_note &n) const;
        bool operator==(const base_note &n) const;
        friend std::ostream &operator<<(std::ostream &os, const base_note &n);

    protected:
        virtual std::string _str() const;

        std::string _func; ///< note function
        std::string _text; ///< note text
        base_note_type _stat; ///< note status

    private:
    };

    // container for gallnotes, should not be derived from gdata as others
    // ----------

    /** @brief class for base_note_all. */
    class base_note_all
    {

    public:
        /** @brief default constructor. */
        base_note_all();

        /** @brief default destructor. */
        virtual ~base_note_all();

        /** @brief std::set/get notes (messages/warning/errors). */
        void mesg(base_note_type note, std::string func, std::string text);

        /** @brief get note (messages/warning/errors). */
        std::vector<base_note> mesg();

        void clear();

    protected:
        mutable base_mutex _gmutex;
        std::vector<base_note> _gnotes; ///< cummulate notes message/warning/error
    };

} // namespace

#endif
