#ifndef optrar_argh_hh
#define optrar_argh_hh

/* Copyright (C) 1992,2004 Bisqwit (http://iki.fi/bisqwit/) */

#include <string>
class ParamHandler
{
private:
    /* No copying */
    void operator=(const ParamHandler &);
    ParamHandler (const ParamHandler &);
    class Reference;
public:
    typedef long keytype;
    
    ParamHandler();
    virtual ~ParamHandler();
    virtual void PrintOpt(unsigned space, const std::string &opts, const std::string &desc);
    
    Reference AddLong(const std::string &longname, keytype alias);
    Reference AddBool(keytype c);
    Reference AddInt(keytype c, int min, int max);
    Reference AddFloat(keytype c, double min, double max);
    Reference AddString(keytype c, unsigned min=1, unsigned max=std::string::npos);
    Reference AddDesc(keytype c, const std::string &s, const std::string &param="");

    // Support for many conventions
    void StartParse(int ac, const char *const *av, int firstarg=1);
    void StartParse(int ac, const char **av, int firstarg=1);
    void StartParse(int ac, char **av, int firstarg=1);
    void StartParse(int ac, char *const*av, int firstarg=1);
    
    const std::string ProgName() const { return A0; }

    /* --not-x, --no-x, --without-x and --with-x are recognized.
     * Therefore you should use GetBool().
     */
    inline const bool GetBool() const      { return polarity; }
    inline const int GetInt() const        { return intparm; }
    inline const double GetFloat() const   { return doubleparm; }
    inline const std::string &GetString() const { return param; }
    inline const bool ok() const           { return !error; }
    
    keytype GetParam();
    void ListOptions();

	/* - disabled, not supported.
public:
     // Keys for 2,3,4 character options with single hyphen
     static inline keytype char2opt(char c1, char c2) { return (unsigned char)c1 + (unsigned char)c2 * 256; }
     static inline keytype char3opt(char c1, char c2, char c3) { return (unsigned char)c1 + 256 * char2opt(c2, c3); }
     static inline keytype char4opt(char c1, char c2, char c3, char c4) { return char2opt(c1, c2) + 65536 * char2opt(c3, c4); }
    */
private:    
    void ErrorIllegalOption(keytype key);
    void ErrorNeedsArg(keytype key);
    void ErrorNeedsArg(const std::string &longo);
    void ErrorNeedNoArg(const std::string &longo);
    void InternalError(keytype key, const char *s);
    void ErrorOutOfRange(const std::string &param);
    void ErrorUnknownOption(const char *s, bool negafail);
        
public:
    /* argh_atypemap needs this and I don't know how to make it friend. */
    class ArgInfo { public: char type; int min,max;unsigned maxl;double fmin,fmax; };
private:
    class argh_aliasmap *aliases;
    class argh_descsmap *descs;
    class argh_atypemap *argtypes;
    class Reference
    {
        ParamHandler& par;
        keytype key;
    public:
        Reference(ParamHandler& p, keytype k) : par(p), key(k) { }
        Reference &SetBool();
        Reference &SetInt(int min, int max);
        Reference &SetFloat(double min, double max);
        Reference &SetString(unsigned min=1, unsigned max=std::string::npos);
        Reference &SetDesc(const std::string &s, const std::string &param="");
    /*- disabled, not needed.
    private:
        Reference(const Reference&);
        const Reference& operator= (const Reference& );
    */
    };
    Reference MakeRef(keytype key);
    int ParseError();
    int ShortOpt(keytype key, const char *s);
private:
    int argc; const char *const *argv;
    const char *A0;
    /* parser volatile */
    int argpos; bool opts; std::string longo, longobuf; std::string param;
    const char *shortpointer; bool error, polarity; int intparm; double doubleparm;
};

#endif
