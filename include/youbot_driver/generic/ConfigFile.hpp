// ConfigFile.hpp
// Class for reading named values from configuration files
// Richard J. Wagner  v2.1  24 May 2004  wagnerr@umich.edu

// Copyright (c) 2004 Richard J. Wagner
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

// Typical usage
// -------------
// 
// Given a configuration file "settings.inp":
//   atoms  = 25
//   length = 8.0  # nanometers
//   name = Reece Surcher
// 
// Named values are read in various ways, with or without default values:
//   ConfigFile config( "settings.inp" );
//   int atoms = config.read<int>( "atoms" );
//   double length = config.read( "length", 10.0 );
//   string author, title;
//   config.readInto( author, "name" );
//   config.readInto( title, "title", string("Untitled") );
// 
// See file example.cpp for more examples.
//***************************************************************************
//	Modified 04 February 2011 by
//	Franz Ehrenhuber, Codronic GmbH
//
//	It's now possible to query sections in config files
//  Default sections are defined like [sectionname],
//	but it is possible to use different tags
//  Section related adding/removing of entries is implemented.
//  Changes have to be saved.
//
//	Examples:
//	youbot::ConfigFile *cfgFile = new youbot::ConfigFile("youbot-manipulator", path);
//  cfgFile->readInto( i1, "Joint_5", "UpperLimit_[encoderTicks]");
//	if(cfgFile->keyExists("Joint_3", "CalibrationDirection"))
//	{
//		cout << "Key CalibrationDirection exists" << endl;
//	}
//	cfgFile->remove("Joint_1", "Test");
//	cfgFile->add("Joint_1", "Test1", 77);
//	cfgFile->save();
//***************************************************************************

#ifndef CONFIGFILE_HPP
#define CONFIGFILE_HPP

#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include "youbot_driver/generic/Exceptions.hpp"

using std::string;

namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// Keep track of sortorder from original configfile
///////////////////////////////////////////////////////////////////////////////
  class SortTreeVector {
  protected:
    string myKey;
    vector<string> mySortVector;

  public:

    SortTreeVector() {
      myKey = "";
    };

    string getKey() {
      if (myKey.empty()) return "";

      if (myKey.size() > 0) {
        return myKey;
      }

      return "";


    }

    void setKey(const string& sKey) {
      myKey = sKey;
    }

    std::vector<string> getVector() const {
      return mySortVector;
    }

    void setVector(const std::vector<string>& vVector) {
      mySortVector = vVector;
    }

    ~SortTreeVector() {
    };

  };

  ///////////////////////////////////////////////////////////////////////////////
  /// Reads and writes a configuration file
  ///////////////////////////////////////////////////////////////////////////////
  class ConfigFile {
    // Data
  protected:
    string myDelimiter; // separator between key and value
    string myComment; // separator between value and comments
    string mySectionStartTag; // tag marks the beginning of a section header
    string mySectionEndTag; // tag marks the end of a section header
    string mySentry; // optional string to signal end of file
    string myFilepath; //Path to Configfile
    SortTreeVector *mySortVectorObj; // Keeps vector for sorting contents and its relation to the Section Key

    std::map<string, string> myContents; // extracted keys and values
    std::map<string, std::map<string, string> > mySectionRelatedContents; // A List of all sections with their extrated key/values

    std::vector<SortTreeVector> mySortVector; // keeps SortTreeVector objects for sorting on Sections an their contents

    typedef std::map<string, string>::iterator mapi;
    typedef std::map<string, string>::const_iterator mapci;
    typedef std::map<string, map<string, string> >::const_iterator mapciSect;

    // Methods
  public:

    ConfigFile(string filename,
            string filepath = "../config/",
            string delimiter = "=",
            string comment = "#",
            string sectionStartTag = "[",
            string sectionEndTag = "]",
            string sentry = "EndConfigFile");
    ConfigFile();
    
    ~ConfigFile(){};

    // Search for key and read value or optional default value
    template<class T> T read(const string& key, const T& value)const;

    //Overload to read key from section lines
    template<class T> T read(const string& key) const; // call as read<T>

    //Overload to read key from section lines
    template<class T> T read(const string& sectionKey, const string& key); // call as read<T>


    //Read key into ref variable
    template<class T> bool readInto(T& var, const string& key) const;
    //Overload to read key from section lines
    template<class T> bool readInto(T& var, const string& sectionKey, const string& key);

    template<class T>
    bool readInto(T& var, const string& key, const T& value) const;

    // Modify keys and values
    template<class T> void add(string key, const T& value);

    // Modify keys and values beyond a sectionkey
    template<class T> void add(string sectionKey, string key, const T& value);


    //Remove key from config file with no sections
    void remove(const string& key);
    //Remove one key from specified section
    void remove(const string& sectionKey, const string& key);

    //Save Changes to Configfile
    //should be invoked after removing/adding keys
    void save();



    // Check whether key exists in configuration
    bool keyExists(const string& key) const;
    // Overload to check key inside a section
    bool keyExists(const string& sectionKey, const string& key);

    //Check for existing section
    bool sectionExists(const string& sectionKey);



    // Check or change configuration syntax

    string getDelimiter() const {
      return myDelimiter;
    }

    string getComment() const {
      return myComment;
    }

    string getSentry() const {
      return mySentry;
    }

    string setDelimiter(const string& s) {
      string old = myDelimiter;
      myDelimiter = s;
      return old;
    }

    string setComment(const string& s) {
      string old = myComment;
      myComment = s;
      return old;
    }

    // Write or read configuration
    friend std::ostream & operator<<(std::ostream& os, ConfigFile& cf);
    friend std::istream & operator>>(std::istream& is, ConfigFile& cf);

  protected:
    template<class T> static string T_as_string(const T& t);
    template<class T> static T string_as_T(const string& s);
    static void trim(string& s);

  };

  /* static */
  template<class T>
  string ConfigFile::T_as_string(const T& t) {
    // Convert from a T to a string
    // Type T must support << operator
    std::ostringstream ost;
    ost << t;
    return ost.str();
  }

  /* static */
  template<class T>
  T ConfigFile::string_as_T(const string& s) {
    // Convert from a string to a T
    // Type T must support >> operator
    T t;
    std::istringstream ist(s);
    ist >> t;
    return t;
  }

  /* static */
  template<>
  inline string ConfigFile::string_as_T<string>(const string& s) {
    // Convert from a string to a string
    // In other words, do nothing
    return s;
  }

  /* static */
  template<>
  inline bool ConfigFile::string_as_T<bool>(const string& s) {
    // Convert from a string to a bool
    // Interpret "false", "F", "no", "n", "0" as false
    // Interpret "true", "T", "yes", "y", "1", "-1", or anything else as true
    bool b = true;
    string sup = s;
    for (string::iterator p = sup.begin(); p != sup.end(); ++p)
      *p = toupper(*p); // make string all caps
    if (sup == string("FALSE") || sup == string("F") ||
            sup == string("NO") || sup == string("N") ||
            sup == string("0") || sup == string("NONE"))
      b = false;
    return b;
  }

  template<class T>
  T ConfigFile::read(const string& key) const {
    mapci p = myContents.find(key);
    if (p == myContents.end()) throw KeyNotFoundException(key);
    return string_as_T<T > (p->second);
  }

  template<class T>
  T ConfigFile::read(const string& sectionKey, const string& key) {
    // Read the value corresponding to key
    mapciSect sp = mySectionRelatedContents.find(sectionKey);
    if (sp == mySectionRelatedContents.end()) throw KeyNotFoundException(sectionKey);

    myContents = sp->second;
    mapci p = myContents.find(key);
    if (p == myContents.end()) throw KeyNotFoundException(key);
    return string_as_T<T > (p->second);
  }

  template<class T>
  T ConfigFile::read(const string& key, const T& value) const {
    // Return the value corresponding to key or given default value
    // if key is not found
    mapci p = myContents.find(key);
    if (p == myContents.end()) return value;
    return string_as_T<T > (p->second);
  }

  template<class T>
  bool ConfigFile::readInto(T& var, const string& key) const {
    // Get the value corresponding to key and store in var
    // Return true if key is found
    // Otherwise leave var untouched

    mapci p = myContents.find(key);
    bool found = (p != myContents.end());
    if (found) {
      var = string_as_T<T > (p->second);
    } else {
      throw KeyNotFoundException(key);
    }
    return found;
  }

  template<class T>
  bool ConfigFile::readInto(T& var, const string& sectionKey, const string& key) {
    // Get the value corresponding to key and store in var
    // Return true if key is found
    // Otherwise leave var untouched

    mapciSect sp = mySectionRelatedContents.find(sectionKey);
    if (sp == mySectionRelatedContents.end()) throw KeyNotFoundException(sectionKey);


    myContents = sp->second;

    mapci p = myContents.find(key);
    bool found = (p != myContents.end());
    if (found) {
      var = string_as_T<T > (p->second);
    } else {
      throw KeyNotFoundException(key);
    }
    return found;
  }

  template<class T>
  void ConfigFile::add(string key, const T& value) {
    // Add a key with given value
    string v = T_as_string(value);
    trim(key);
    trim(v);
    //Check for dublicate keys
    mapi p = myContents.find(key);
    if (p != myContents.end()) {
      return;
    }
    myContents[key] = v;
    return;
  }

  template<class T>
  void ConfigFile::add(string sectionKey, string key, const T& value) {


    // Add a key with given value
    string v = T_as_string(value);
    trim(key);
    trim(v);

    mapciSect sp = mySectionRelatedContents.find(sectionKey);

    //Write SectionKey  with values if no section found
    if (sp == mySectionRelatedContents.end()) {
      SortTreeVector vsort;
      vector<string> vNewVal;
      map<string, string> newMap;
      newMap[key] = v;
      vsort.setKey(sectionKey);
      vNewVal.push_back(key);
      vsort.setVector(vNewVal);
      mySectionRelatedContents[sectionKey] = newMap;
      mySortVector.push_back(vsort);
      return;
    }

    myContents = sp->second;

    //Check for dublicate keys
    mapi p = myContents.find(key);
    if (p != myContents.end()) {
      return;
    }

    myContents[key] = v;
    mySectionRelatedContents[sectionKey] = myContents;

    for (unsigned int i = 0; i < mySortVector.size(); i++) {

      if (mySortVector[i].getKey() == sectionKey) {
        vector<string> sortVec = mySortVector[i].getVector();
        sortVec.push_back(key);
        mySortVector[i].setVector(sortVec);


      }
    }


    return;
  }


} // namespace youbot

#endif  // CONFIGFILE_HPP

// Release notes:
// v1.0  21 May 1999
//   + First release
//   + Template read() access only through non-member readConfigFile()
//   + ConfigurationFileBool is only built-in helper class
// 
// v2.0  3 May 2002
//   + Shortened name from ConfigurationFile to ConfigFile
//   + Implemented template member functions
//   + Changed default comment separator from % to #
//   + Enabled reading of multiple-line values
// 
// v2.1  24 May 2004
//   + Made template specializations inline to avoid compiler-dependent linkage
//   + Allowed comments within multiple-line values
//   + Enabled blank line termination for multiple-line values
//   + Added optional sentry to detect end of configuration file
//   + Rewrote messy trimWhitespace() function as elegant trim()
//
// v2.2 04 February 2011
//	+ Added Section support
//	+ Check for duplicate key in add-Method
//	+ Removed KeyNotFoundexception in remove-Method because it is
//	  more convenient not to interrupt if key does not exists.

