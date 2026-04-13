#include "hwa_base_note.h"
#include "hwa_base_common.h"
#include "hwa_base_typeconv.h"

namespace hwa_base
{
    // constructor
    // ---------
    base_note::base_note(base_note_type n, std::string f, std::string s)
    {

        _stat = n;
        _func = f;
        _text = s;
    }

    // destructor
    // ---------
    base_note::~base_note()
    {
    }

    // overloading << operator
    // -----------------------------
    std::ostream &operator<<(std::ostream &os, const base_note &n)
    {
        os << n.str();
        return os;
    }

    // overloading == operator
    // -----------------------------
    bool base_note::operator==(const base_note &n) const
    {
        //  boost::mutex::scoped_lock lock(_mutex_Triple);
        return (n.status() == _stat &&
                n.func() == _func &&
                n.text() == _text);
    }

    // overloading < operator
    // -----------------------------
    bool base_note::operator<(const base_note &n) const
    {
        //  boost::mutex::scoped_lock lock(_mutex_Triple);
        return (n.status() < _stat &&
                n.func() < _func &&
                n.text() < _text);
    }

    // get std::string
    // ---------
    std::string base_note::_str() const
    {
        std::string note;
        switch (_stat)
        {
        case GMESSAGE:
            note = "Message - ";
            break;
        case GWARNING:
            note = "Warning - ";
            break;
        case GERROR:
            note = "Error - ";
            break;
        }

        return note;
    }

    // constructor
    // ----------
    base_note_all::base_note_all()
    {
    }

    // destructor
    // ----------
    base_note_all::~base_note_all()
    {

        this->clear();
    }

    // clean
    // --------------------
    void base_note_all::clear()
    {
        _gnotes.clear();
    }

    // add note (messages/warning/errors)
    // ---------------------
    void base_note_all::mesg(base_note_type note, std::string func, std::string text)
    {
        base_note gnote(note, func, text);

        // eliminate repeating messages
        bool exist = false;
        for (auto it = _gnotes.begin(); it != _gnotes.end(); ++it)
        {
            if (*it == gnote)
            {
                exist = true;
            }
        }
        if (!exist)
            _gnotes.push_back(gnote);
        return;
    }

    // get note (messages/warning/errors)
    // --------------------
    std::vector<base_note> base_note_all::mesg()
    {

        return _gnotes;
    }

} // namespace