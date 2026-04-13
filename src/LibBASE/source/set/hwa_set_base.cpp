#include "hwa_set_base.h"
using namespace pugi;

namespace hwa_set
{
    set_base::set_base()
        : _name(""),
        _pgm(""),
        _ver(""),
        _rev(""),
        _own(""),
        _dat(""),
        _tim("")
    {
        _delimiter = "  ";
        _set.insert("base");
    }

    set_base::~set_base()
    {
    }

    // Read from file
    // ----------
    int set_base::read(const std::string& file)
    {
        _name = file;

        if (!(_irc = _doc.load_file(_name.c_str())))
        {
            //    if( _log ) _log->comment(0, "gsetbase" , "xconfig: not file read " + _name + " " + std::string(_irc.description()) );
            //    else                             std::cerr << "xconfig: not file read " + _name + " " + std::string(_irc.description()) << std::endl;
            std::cerr << "xconfig: not file read " + _name + " " + std::string(_irc.description()) << std::endl;
            return -1;
        }

        //  if( _log ) _log->comment(0, "gsetbase" , "xconfig: read from file " + _name );
        this->check();

        return 0;
    }

    // Read from istream
    // ----------
    int set_base::read_istream(std::istream& is)
    {
        //#if defined _WIN32 || defined _WIN64
        //        if (_isatty(_fileno(stdin)))
        //            return -1; // stdin is terminal
        //#else                  // pipe
        //        if (isatty(fileno(stdin)))
        //            return -1; // stdin is terminal
        //                       // else return 1;
        //#endif
        //        return 1; // NOT FUNCTIONAL

                // ELSE: stdin is pipe (or file)
        if (!(_irc = _doc.load(is)))
        {
            //    if( _log ) _log->comment(0, "gsetbase" , "xconfig read: not istream! [" + std::string(_irc.description()) + "]" );
            //    else                             std::cerr << "xconfig read: not istream! [" + std::string(_irc.description()) + "]" << std::endl;
            std::cerr << "xconfig read: not istream! [" + std::string(_irc.description()) + "]" << std::endl;
            return 1;
        }

        //  if( _log ) _log->comment(0, "gsetbase" , "xconfig read: istream");
        this->check();

        return 0;
    }

    // Write to file
    // ----------
    int set_base::write(const std::string& file)
    {
        std::ofstream of;
        std::string name(file); // if( name.empty() ) name = _name;  // ???
        try
        {
            of.open(name.c_str());
        }
        catch (std::fstream::failure e)
        {
            std::cerr << "Warning: Exception opening file " << name << ": " << e.what() << std::endl;
            return -1;
        }

        _doc.save_file(name.c_str(), _delimiter.c_str(), pugi::format_indent);

        of.close();

        std::cout << "XML-config saved: " + name << std::endl;
        return 0;
    }

    // Write to std::ostream
    // ----------
    int set_base::write_ostream(std::ostream& os)
    {
        _doc.save(os, _delimiter.c_str(), pugi::format_indent);
        return 0;
    }

    // Return first std::string of value based on key
    // ----------
    std::string set_base::strval(const std::string& elem, const std::string& subelem)
    {
        std::string tmp = _strval(elem, subelem);
        return tmp;
    }

    // application usage
    // ----------
    void set_base::usage()
    {
        std::cout << std::endl
            << app() << std::endl;
        std::cout << std::endl
            << "Usage: "
            << std::endl
            << std::endl
            << "    -h|--help              .. this help                          "
            << std::endl
            << "    -V int                 .. version                            "
            << std::endl
            << "    -v int                 .. verbosity level                    "
            << std::endl
            << "    -x file                .. configuration input file           "
            << std::endl
            << "    --                     .. configuration from stdinp          "
            << std::endl
            << "    -l file                .. spdlog output file                    "
            //     << std::endl << "    -m                     .. multi-threaded processing          "
            << std::endl
            << "    -X                     .. output default configuration in XML"
            << std::endl
            << std::endl;

        exit(0);
        return;
    }

    // Use command-line arguments
    // ----------
    void set_base::arg(int argc, char* argv[], bool add, bool thin)
    {
        std::string conf("");
        std::string save("");
        // bool stdi = false; // ENFORCE STDINP

        // std::cerr << "command-line opt: " << argc << std::endl;  for( int i=1; i<argc; ++i ) std::cerr << argv[i] << " "; std::cerr << std::endl;

        for (int i = 1; i < argc; ++i)
        {
            if (!strcmp(argv[i], "--help") && i < argc)
            {
                usage();
                exit(0);
            }
            else if (!strcmp(argv[i], "-h") && i < argc)
            {
                usage();
                exit(0);
            }
            else if (!strcmp(argv[i], "-X") && i < argc)
            {
                help();
                exit(0);
            }
            //  else if( !strcmp(argv[i],"-m") && i  <argc ){ thrd = true; }
            else if (!strcmp(argv[i], "-x") && i + 1 < argc)
            {
                conf = argv[++i];
            }
            else if (!strcmp(argv[i], "-Z") && i + 1 < argc)
            {
                save = argv[++i];
            }
            //  else if( !strcmp(argv[i],"-v") && i+1<argc ){ if(_log){ _log->verb(atoi(argv[++i])); }} // ZDE PRILIS BRZY!!
            //  else if( !strcmp(argv[i],"-l") && i+1<argc ){ if(_log){ _log->mask(     argv[++i]);  }} // ZDE PRILIS BRZY!!

            // warning if no other commandline option is supported
            else if (!add)
                fprintf(stderr, "Unknown option: %s\n", argv[i]);
        }

        if (!conf.empty())
        {
            if (read(conf))
            {
                exit(1);
            } // file XML
        }
        else
        {
            exit(1);
        }

        // add/replace command-line option in XML
        for (int i = 1; i < argc; ++i)
        {
            xml_node NODE = _default_node(_doc, "config", "", true);

            std::string s = argv[i];
            size_t pos1 = 0, pos2 = 0;
            if (s[0] == '-')
                continue;

            //TODO COMMENT
            bool isNode = s.find("node") != std::string::npos;
            bool isAttr = s.find("attr") != std::string::npos;
            // É¾³ý node£ºor attr:
            if (isNode || isAttr)
                s = s.substr(5);

            while (((pos1 = s.find(':', pos2)) != std::string::npos || // i+1 < argc ... used later
                (pos1 = s.find('=', pos2)) != std::string::npos))  // ORDER IMPORTANT
            {
                if ((pos2 = s.find(':', pos1 + 1)) != std::string::npos)
                {
                    NODE = _default_node(NODE, s.substr(pos1 + 1, pos2 - pos1 - 1).c_str(), "", true);
                }
                else if ((pos2 = s.find('=', pos1 + 1)) != std::string::npos)
                {
                    if (isAttr)
                        _default_attr(NODE, s.substr(pos1 + 1, pos2 - pos1 - 1).c_str(), s.substr(pos2 + 1, std::string::npos), true);
                    else if (isNode)
                        _default_node(NODE, s.substr(pos1 + 1, pos2 - pos1 - 1).c_str(), s.substr(pos2 + 1, std::string::npos).c_str(), true);
                    else
                        _default_node(NODE, s.substr(pos1 + 1, pos2 - pos1 - 1).c_str(), s.substr(pos2 + 1, std::string::npos).c_str(), true);

                    pos2 = std::string::npos;
                }
                else
                {
                    if (i + 1 < argc)
                        _default_node(NODE, s.substr(pos1 + 1, std::string::npos).c_str(), argv[++i], true);
                    else
                        std::cerr << "Incomplete command-line argument\n";
                    pos2 = std::string::npos;
                }
            }
        }

        if (thin)
        {
            xml_node node = _doc.child(XMLKEY_ROOT);
            _default_attr(node, "thin", "true", true);
        }
        //£¿£¿
        this->check();

        if (!conf.empty())
        {
            std::cout << "xconfig: read from file " + _name << std::endl;
        }
        else
        {

            std::cout << "xconfig: read from istream" << std::endl;
        }

        if (!save.empty())
        {
            std::cout << "xconfig: save to file " + _name << std::endl;
            write(save);
        }

        // COMMENTS

        return;
    }

    // Set app information
    // ----------
    void set_base::app(const std::string& pgm, const std::string& ver,
        const std::string& rev, const std::string& own,
        const std::string& dat, const std::string& tim)
    {
        _pgm = pgm;
        _ver = ver;
        _rev = rev;
        _own = own;
        _dat = dat;
        _tim = tim;
        return;
    }

    // Return app information
    // ----------
    std::string set_base::app()
    {
        std::string tmp = _pgm + " [" + _ver + "] compiled: " + _dat + " " + _tim + " (" + _rev + ")";
        return tmp;
    };

    // Return app information
    // ----------
    std::string set_base::pgm()
    {
        std::string tmp = _pgm;
        return tmp;
    };

    // Return std::set of value based on key
    // ----------
    std::set<std::string> set_base::setval(const std::string& elem, const std::string& subelem)
    {
        std::set<std::string> tmp = _setval(elem, subelem);
        return tmp;
    }

    std::set<std::string> set_base::setvals(const std::string& elem, const std::string& subelem)
    {
        std::set<std::string> tmp = _setvals(elem, subelem);
        return tmp;
    }

    std::map<std::string, std::set<std::string>> set_base::setval_map(const std::string& elem, const std::string& subelem, const std::string& subelem1, const char* attri)
    {
        std::map<std::string, std::set<std::string>> tmp = _setval_map(elem, subelem, subelem1, attri);
        return tmp;
    }

    void set_base::setval(const std::string& elem, const std::string& subelem, const std::string& val)
    {
        this->_setval(elem, subelem, val);
    }

    void set_base::setval(const std::string& elem, const std::string& subelem, const std::string& sub_subelem, const std::string& val)
    {
        this->_setval(elem, subelem, sub_subelem, val);
    }

    // Return std::vector of value based on key
    // ----------
    std::vector<std::string> set_base::vecval(const std::string& elem, const std::string& subelem)
    {
        std::vector<std::string> tmp = _vecval(elem, subelem);
        return tmp;
    }

    // Return value
    // ----------
    bool set_base::thin()
    {
        bool tmp = _doc.child(XMLKEY_ROOT).attribute("thin").as_bool();
        return tmp;

        // print log from the chache
        // --------------
    }

    /// get the node of confige
    xml_node set_base::xml_root_node()
    {
        return _doc.child("config");
    }

    /// get the node of confige
    xml_node set_base::xml_config_node(std::string node_name)
    {
        return _doc.child("config").child(node_name.c_str());
    }

    // Return first double of attribute based on key
    // ----------
    double set_base::_dblatt(const std::string& elem, const std::string& attrib)
    {
        double num = 0.0;

        num = _doc.child(XMLKEY_ROOT).child(elem.c_str()).attribute(attrib.c_str()).as_double();

        return num;
    }

    double set_base::_dblatt(const std::string& elem, const std::string& sub_elem, const std::string& attrib)
    {
        double num = 0.0;

        num = _doc.child(XMLKEY_ROOT).child(elem.c_str()).child(sub_elem.c_str()).attribute(attrib.c_str()).as_double();

        return num;
    }

    // Return first double of value based on key
    // ----------
    double set_base::_dblval(const std::string& elem, const std::string& subelem)
    {
        double num = 0.0;

        std::istringstream is(_doc.child(XMLKEY_ROOT).child(elem.c_str()).child_value(subelem.c_str()));

        if (is >> num)
        {
            return num;
        }

        return num;
    }

    // Return first std::string of value based on key
    // ----------
    std::string set_base::_strval(const std::string& elem, const std::string& subelem)
    {
        std::string word;

        std::istringstream is(_doc.child(XMLKEY_ROOT).child(elem.c_str()).child_value(subelem.c_str()));

        if (is >> word)
        {
            transform(word.begin(), word.end(), word.begin(), ::toupper);
            return word;
        }

        return "";
    }

    // Return std::set of value based on key
    // ----------
    std::set<std::string> set_base::_setval(const std::string& elem, const std::string& subelem)
    {
        std::set<std::string> vals;
        std::string word;
        //std::string ELEM = elem; transform(ELEM.begin(), ELEM.end(), ELEM.begin(), ::toupper);
        std::istringstream is(_doc.child(XMLKEY_ROOT).child(elem.c_str()).child_value(subelem.c_str()));
        while (is >> word)
        {
            transform(word.begin(), word.end(), word.begin(), ::toupper);
            vals.insert(word);
        }

        return vals;
    }

    std::set<std::string> set_base::_setvals(const std::string& elem, const std::string& subelem, const std::string& sub_subelem)
    {
        std::set<std::string> vals;
        std::string temp;
        std::string word;
        auto xml_node = _doc.child(XMLKEY_ROOT).child(elem.c_str()).child(subelem.c_str()).children(sub_subelem.c_str());
        for (auto rec_node = xml_node.begin(); rec_node != xml_node.end(); rec_node++)
        {
            temp += rec_node->child_value();
        }
        std::istringstream is(temp);
        while (is >> word)
        {
            transform(word.begin(), word.end(), word.begin(), ::toupper);
            vals.insert(word);
        }

        return vals;
    }

    void set_base::_setval(const std::string& elem, const std::string& subelem, const std::string& val)
    {
        _doc.child(XMLKEY_ROOT).child(elem.c_str()).child(subelem.c_str()).set_value(val.c_str());
    }

    void set_base::_setval(const std::string& elem, const std::string& subelem, const std::string& sub_subelem, const std::string& val)
    {
        _doc.child(XMLKEY_ROOT).child(elem.c_str()).child(subelem.c_str()).child(sub_subelem.c_str()).set_value(val.c_str());
    }

    std::set<std::string> set_base::_setvals(const std::string& elem, const std::string& subelem)
    {
        std::set<std::string> vals;
        std::string temp;
        std::string word;
        //std::string ELEM = elem; transform(ELEM.begin(), ELEM.end(), ELEM.begin(), ::toupper);
        auto xml_node = _doc.child(XMLKEY_ROOT).child(elem.c_str()).children(subelem.c_str());
        for (auto rec_node = xml_node.begin(); rec_node != xml_node.end(); rec_node++)
        {
            temp += rec_node->child_value();
        }
        std::istringstream is(temp);
        while (is >> word)
        {
            transform(word.begin(), word.end(), word.begin(), ::toupper);
            vals.insert(word);
        }

        return vals;
    }

    std::map<std::string, std::set<std::string>> set_base::_setval_map(const std::string& elem, const std::string& subelem, const std::string& subelem1, const char* attri)
    {
        std::map<std::string, std::set<std::string>> valmap;
        std::string temp, tag, word;
        auto xml_node = _doc.child(XMLKEY_ROOT).child(elem.c_str()).child(subelem.c_str()).children(subelem1.c_str());
        for (auto rec_node = xml_node.begin(); rec_node != xml_node.end(); rec_node++)
        {
            std::set<std::string> vals;
            temp = rec_node->child_value();
            tag = rec_node->attribute(attri).value();
            std::istringstream is(temp);
            while (is >> word)
            {
                transform(word.begin(), word.end(), word.begin(), ::toupper);
                vals.insert(word);
            }
            valmap[tag] = vals;
        }
        return valmap;
    }

    std::map<std::string, std::set<std::string>> set_base::_setval_map(const std::string& elem, const std::string& subelem, const std::string& subelem1, const std::string& subelem2, const char* attri)
    {
        std::map<std::string, std::set<std::string>> valmap;
        std::string temp, tag, word;
        auto xml_node = _doc.child(XMLKEY_ROOT).child(elem.c_str()).child(subelem.c_str()).child(subelem1.c_str()).children(subelem2.c_str());
        for (auto rec_node = xml_node.begin(); rec_node != xml_node.end(); rec_node++)
        {
            std::set<std::string> vals;
            temp = rec_node->child_value();
            tag = rec_node->attribute(attri).value();
            std::istringstream is(temp);
            while (is >> word)
            {
                transform(word.begin(), word.end(), word.begin(), ::toupper);
                vals.insert(word);
            }
            valmap[tag] = vals;
        }
        return valmap;
    }

    std::set<std::string> set_base::_setval_attribute(const std::string& elem, const std::string& subelem, const std::string& attri_name, const std::string& attri_value)
    {
        std::istringstream is(_doc.child(XMLKEY_ROOT).child(elem.c_str()).child(subelem.c_str()).find_child_by_attribute(attri_name.c_str(), attri_value.c_str()).child_value());
        std::set<std::string> vals;
        std::string word;
        while (is >> word)
        {
            transform(word.begin(), word.end(), word.begin(), ::toupper);
            vals.insert(word);
        }

        return vals;
    }

    std::set<std::string> set_base::_setval_attribute(const std::string& elem, const std::string& subelem, const std::string& sub_subelem, const std::string& attri_name, const std::string& attri_value)
    {
        std::istringstream is(_doc.child(XMLKEY_ROOT).child(elem.c_str()).child(subelem.c_str()).child(sub_subelem.c_str()).find_child_by_attribute(attri_name.c_str(), attri_value.c_str()).child_value());
        std::set<std::string> vals;
        std::string word;
        while (is >> word)
        {
            transform(word.begin(), word.end(), word.begin(), ::toupper);
            vals.insert(word);
        }

        return vals;
    }

    // Return std::vector of value based on key
    // ----------
    std::vector<std::string> set_base::_vecval(const std::string& elem, const std::string& subelem)
    {
        std::vector<std::string> vec;
        std::string word;

        std::istringstream is(_doc.child(XMLKEY_ROOT).child(elem.c_str()).child_value(subelem.c_str()));
        while (is >> word)
        {
            transform(word.begin(), word.end(), word.begin(), ::toupper);
            vec.push_back(word);
        }

        return vec;
    }

    std::vector<std::string> set_base::_vecval(const std::string& elem, const std::string& subelem, const std::string& sub_subelem)
    {
        std::vector<std::string> vec;
        std::string word;

        std::istringstream is(_doc.child(XMLKEY_ROOT).child(elem.c_str()).child(subelem.c_str()).child_value(sub_subelem.c_str()));
        while (is >> word)
        {
            transform(word.begin(), word.end(), word.begin(), ::toupper);
            vec.push_back(word);
        }

        return vec;
    }

    // check/std::set node
    // ----------
    xml_node set_base::_default_node(xml_node& node, const char* n, const char* val, bool reset)
    {

#ifdef DEBUG
        std::cout << "creating node [" << n << "]  for parent [" << node.name() << "]  std::set with value [" << val << "]\n";
#endif

        std::string s(n);
        transform(s.begin(), s.end(), s.begin(), ::tolower);

        // remove if to be reset
        if (strcmp(val, "") && reset)
            node.remove_child(s.c_str());

#ifdef DEBUG
        if (!node)
            std::cerr << " NODE[" << node.name() << "] " << node << " does'not exists !\n";
        else
            std::cerr << " NODE[" << node.name() << "] " << node << " does exists .... ok \n";
#endif

        xml_node elem = node.child(s.c_str());

#ifdef DEBUG
        if (!elem)
            std::cerr << " ELEM[" << s << "] " << elem << " does'not exists !\n";
        else
            std::cerr << " ELEM[" << s << "] " << elem << " does exists .... ok \n";
#endif

        if (!elem)
        {
            reset = true;
            elem = node.append_child(s.c_str());
        }

        if (!elem)
        {

            std::cout << "warning - cannot create element " + s << std::endl;
        }

        if (elem && strcmp(val, "") && reset)
        {
            elem.append_child(pugi::node_pcdata).set_value(val);
        }

        return elem;
    }

    // check attributes & std::set default
    // ----------
    void set_base::_default_attr(xml_node& node, const char* n, const std::string& v, bool reset)
    {
        if (node.attribute(n).empty())
            node.append_attribute(n);
        if (strlen(node.attribute(n).value()) == 0 || reset)
            node.attribute(n).set_value(v.c_str());

        std::string s = node.attribute(n).value();
        transform(s.begin(), s.end(), s.begin(), ::tolower);
        node.attribute(n).set_value(s.c_str());
    }

    // check attributes & std::set default
    // ----------
    void set_base::_default_attr(xml_node& node, const char* n, const bool& v, bool reset)
    {
        if (node.attribute(n).empty())
            node.append_attribute(n);
        if (strlen(node.attribute(n).value()) == 0 || reset)
            node.attribute(n).set_value(v);
    }

    // check attributes & std::set default
    // ----------
    void set_base::_default_attr(xml_node& node, const char* n, const int& v, bool reset)
    {
        if (node.attribute(n).empty())
            node.append_attribute(n);
        if (strlen(node.attribute(n).value()) == 0 || reset)
            node.attribute(n).set_value(v);
    }

    // check attributes & std::set default
    // ----------
    void set_base::_default_attr(xml_node& node, const char* n, const double& v, bool reset)
    {
        if (node.attribute(n).empty())
            node.append_attribute(n);
        if (strlen(node.attribute(n).value()) == 0 || reset)
            node.attribute(n).set_value(v);
    }

    // add warning log into the chache
    // ---------
    void set_base::_add_log(std::string element, std::string msg)
    {
        std::map<std::string, std::set<std::string>>::iterator itELEM = _chache_log.find(element);
        if (itELEM != _chache_log.end())
        {
            itELEM->second.insert(msg);
        }
        else
        {
            std::set<std::string> msgs;
            msgs.insert(msg);
            _chache_log[element] = msgs;
        }
    }

    // XML help header (optional text comment)
    // ----------
    void set_base::help_header()
    {
        std::cerr << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?> \n"
            << "<!DOCTYPE config>\n\n"
            << "<config>\n";
    }

    // XML help footer (optional text comment)
    // ----------
    void set_base::help_footer()
    {
        std::cerr << "</config>\n";
    }

    void set_base::str_erase(std::string& str)
    {
        str.erase(remove(str.begin(), str.end(), ' '), str.end());
        str.erase(remove(str.begin(), str.end(), '\t'), str.end());
        str.erase(remove(str.begin(), str.end(), '\n'), str.end());
    }
} // namespace
