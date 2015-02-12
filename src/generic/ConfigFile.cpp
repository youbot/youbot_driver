// ConfigFile.cpp

#include "youbot_driver/generic/ConfigFile.hpp"

using std::string;

namespace youbot {

  ConfigFile::ConfigFile(string filename, string filepath, string delimiter,
          string comment, string sectionStartTag, string sectionEndTag, string sentry)
  : myDelimiter(delimiter), myComment(comment), mySectionStartTag(sectionStartTag), mySectionEndTag(sectionEndTag), mySentry(sentry) {

    if (filepath.length() > 0 && filepath.at(filepath.length() - 1) != '/') {
      filepath.append("/");
    }
    filepath.append(filename);

    //Ensure, that we are working with Config Files
    if (filepath.substr(filepath.length() - 4, 4) != ".cfg")
      filepath.append(".cfg");

    //Store for later use
    myFilepath = filepath;
    mySortVectorObj = new SortTreeVector();

    // Construct a ConfigFile, getting keys and values from given file
    std::ifstream in(filepath.c_str());

    if (!in) throw FileNotFoundException(filepath);

    in >> (*this);
  }

  ConfigFile::ConfigFile()
  : myDelimiter(string(1, '=')), myComment(string(1, '#')), mySectionStartTag(string(1, '[')), mySectionEndTag(string(1, ']')) {
    // Construct a ConfigFile without a file; empty
  }

  void ConfigFile::remove(const string& key) {
    // Remove key and its value
    myContents.erase(myContents.find(key));
    return;
  }

  void ConfigFile::remove(const string& sectionKey, const string& key) {
    mapciSect sp = mySectionRelatedContents.find(sectionKey);
    if (sp != mySectionRelatedContents.end()) {
      // Remove key and its value
      myContents = sp->second;

      std::cout << "Size: " << myContents.size() << endl;


      mapi p = myContents.find(key);
      if (p == myContents.end()) throw KeyNotFoundException(key);
      myContents.erase(p);
      mySectionRelatedContents[sectionKey] = myContents;
    }
    return;
  }

  void ConfigFile::save() {

    ofstream outFile(myFilepath.c_str());

    outFile << (*this);

  }

  bool ConfigFile::keyExists(const string& key) const {
    // Indicate whether key is found
    mapci p = myContents.find(key);
    return ( p != myContents.end());
  }

  bool ConfigFile::keyExists(const string& sectionKey, const string& key) {
    mapciSect sp = mySectionRelatedContents.find(sectionKey);
    if (sp == mySectionRelatedContents.end()) {
      return false;
    }

    // Indicate whether key is found
    mapci p = mySectionRelatedContents[sectionKey].find(key);
    return ( p != mySectionRelatedContents[sectionKey].end());
  }

  bool ConfigFile::sectionExists(const string& sectionKey) {

    mapciSect sp = mySectionRelatedContents.find(sectionKey);
    if (sp == mySectionRelatedContents.end()) {
      return false;
    }
    return true;
  }

  /* static */
  void ConfigFile::trim(string& s) {
    // Remove leading and trailing whitespace
    static const char whitespace[] = " \n\t\v\r\f";
    s.erase(0, s.find_first_not_of(whitespace));
    s.erase(s.find_last_not_of(whitespace) + 1U);
  }

  std::ostream & operator<<(std::ostream& os, ConfigFile& cf) {

    for (unsigned int i = 0; i < cf.mySortVector.size(); i++) {

      SortTreeVector currentTreeVector = cf.mySortVector.at(i);
      string currentSector = currentTreeVector.getKey();

      ConfigFile::mapciSect sp = cf.mySectionRelatedContents.find(currentSector.c_str());
      //First section has no empty line
      if (i > 0) {
        os << endl;
      }
      os << "[" << sp->first << "]" << endl;

      cf.myContents = sp->second;

      std::vector<string> contentVector = currentTreeVector.getVector();
      for (unsigned int j = 0; j < contentVector.size(); j++) {
        string currentKey = contentVector.at(j);

        // Save a ConfigFile to os
        ConfigFile::mapci p = cf.myContents.find(currentKey);

        if (p != cf.myContents.end()) {
          os << p->first << " " << cf.myDelimiter << " ";
          os << p->second << std::endl;
        }
      }
    }

    return os;

  }

  std::istream & operator>>(std::istream& is, ConfigFile& cf) {
    // Load a ConfigFile from is
    // Read in keys and values, keeping internal whitespace
    typedef string::size_type pos;
    const string& delim = cf.myDelimiter; // separator
    const string& comm = cf.myComment; // comment
    const string& startTag = cf.mySectionStartTag; // starttag
    const string& endTag = cf.mySectionEndTag; // endtag
    const string& sentry = cf.mySentry; // end of file sentry
    const pos skip = delim.length(); // length of separator



    string nextline = ""; // might need to read ahead to see where value ends
    string sectHeader = "";
    std::vector<string> currentVector; //holds key-value pairs for each section


    while (is || nextline.length() > 0) {

      // Read an entire line at a time
      string line;
      if (nextline.length() > 0) {
        line = nextline; // we read ahead; use it now
        nextline = "";
      } else {
        std::getline(is, line);
      }

      // Ignore comments
      line = line.substr(0, line.find(comm));

      string lncopy = line;
      ConfigFile::trim(lncopy);
      //Sectionheader found
      if (lncopy.substr(0, 1) == startTag && lncopy.substr(lncopy.length() - 1, 1) == endTag) {

        sectHeader = lncopy.substr(1, lncopy.length() - 2);
        ConfigFile::trim(sectHeader);
        //New instance for each section
        string currentKey = cf.mySortVectorObj->getKey();

        if (!cf.mySortVectorObj || sectHeader != currentKey) {

          if (currentVector.size() > 0) {
            cf.mySortVectorObj->setVector(currentVector);
            cf.mySortVector.push_back(*cf.mySortVectorObj);

          }
          cf.mySortVectorObj = new SortTreeVector;
          //Create a new vector for KeyValue-Pairs
          currentVector = cf.mySortVectorObj->getVector();
          cf.mySortVectorObj->setKey(sectHeader);
        }

        cf.mySectionRelatedContents[sectHeader] = cf.myContents;
        continue;
      }

      // Check for end of file sentry
      if (sentry != "" && line.find(sentry) != string::npos) return is;

      // Parse the line if it contains a delimiter
      pos delimPos = line.find(delim);
      if (delimPos < string::npos) {
        // Extract the key
        string key = line.substr(0, delimPos);
        line.replace(0, delimPos + skip, "");

        // See if value continues on the next line
        // Stop at blank line, next line with a key, end of stream,
        // or end of file sentry
        bool terminate = false;
        while (!terminate && is) {
          std::getline(is, nextline);
          terminate = true;

          string nlcopy = nextline;
          ConfigFile::trim(nlcopy);
          if (nlcopy == "") continue;

          nextline = nextline.substr(0, nextline.find(comm));
          if (nextline.find(delim) != string::npos)
            continue;
          if (sentry != "" && nextline.find(sentry) != string::npos)
            continue;

          nlcopy = nextline;
          ConfigFile::trim(nlcopy);
          if (nlcopy != "") line += "\n";
          line += nextline;
          terminate = false;
        }

        // Store key and value
        ConfigFile::trim(key);
        ConfigFile::trim(line);

        currentVector.push_back(key);

        cf.mySectionRelatedContents[sectHeader][key] = line; // overwrites if key is repeated
      }
    }

    //finally get the last things done
    if (currentVector.size() > 0) {
      cf.mySortVectorObj->setVector(currentVector);
      cf.mySortVector.push_back(*cf.mySortVectorObj);
    }


    return is;
  }

} // namespace youbot
