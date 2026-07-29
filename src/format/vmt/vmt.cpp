#include "vmt.h"

#include <algorithm>
#include <cctype>

namespace format
{
    namespace
    {
        std::string lower(std::string v)
        {
            std::transform(v.begin(), v.end(), v.begin(),
                           [](unsigned char c) { return (char)std::tolower(c); });
            return v;
        }

        // a tiny keyvalues tokenizer: quoted or bare tokens, // line comments,
        // and the { } block delimiters as single-character tokens.
        class tokenizer
        {
        public:
            tokenizer(const char *data, std::size_t size) : p_(data), end_(data + size) {}

            bool next(std::string &token)
            {
                for (;;)
                {
                    while (p_ < end_ && std::isspace((unsigned char)*p_))
                        p_++;
                    if (p_ + 1 < end_ && p_[0] == '/' && p_[1] == '/')
                    {
                        while (p_ < end_ && *p_ != '\n')
                            p_++;
                        continue;
                    }
                    break;
                }
                if (p_ >= end_)
                    return false;

                if (*p_ == '{' || *p_ == '}')
                {
                    token.assign(1, *p_++);
                    return true;
                }
                if (*p_ == '"')
                {
                    p_++;
                    const char *start = p_;
                    while (p_ < end_ && *p_ != '"')
                        p_++;
                    token.assign(start, p_);
                    if (p_ < end_)
                        p_++;
                    return true;
                }
                const char *start = p_;
                while (p_ < end_ && !std::isspace((unsigned char)*p_) && *p_ != '{'
                       && *p_ != '}' && *p_ != '"')
                    p_++;
                token.assign(start, p_);
                return true;
            }

        private:
            const char *p_;
            const char *end_;
        };

        // consumes tokens until the block opened before this call is balanced.
        void skip_block(tokenizer &tk)
        {
            int depth = 1;
            std::string token;
            while (depth > 0 && tk.next(token))
            {
                if (token == "{")
                    depth++;
                else if (token == "}")
                    depth--;
            }
        }
    }

    bool vmt_material::is_patch() const
    {
        return lower(shader) == "patch";
    }

    std::string vmt_material::patch_include() const
    {
        return get("include");
    }

    std::string vmt_material::get(const std::string &key) const
    {
        auto it = values.find(lower(key));
        return it == values.end() ? std::string{} : it->second;
    }

    bool parse_vmt(const std::vector<byte> &text, vmt_material &out, std::string *error)
    {
        out = vmt_material{};
        tokenizer tk((const char *)text.data(), text.size());

        std::string token;
        if (!tk.next(token))
        {
            if (error)
                *error = "empty vmt";
            return false;
        }
        out.shader = token;

        if (!tk.next(token) || token != "{")
        {
            if (error)
                *error = "vmt is missing its material block";
            return false;
        }

        // read key/value pairs; a value of "{" opens a nested block we skip. for a
        // patch material the insert/replace blocks are flattened so their keys
        // (e.g. $basetexture) surface at the top level.
        while (tk.next(token))
        {
            if (token == "}")
                break;
            std::string key = token;
            std::string value;
            if (!tk.next(value))
                break;
            if (value == "}")
                break;
            if (value == "{")
            {
                std::string lkey = lower(key);
                if (lkey == "insert" || lkey == "replace")
                {
                    std::string k2, v2;
                    while (tk.next(k2) && k2 != "}")
                    {
                        if (!tk.next(v2) || v2 == "}")
                            break;
                        if (v2 == "{")
                        {
                            skip_block(tk);
                            continue;
                        }
                        out.values[lower(k2)] = v2;
                    }
                }
                else
                {
                    skip_block(tk);
                }
                continue;
            }
            out.values[lower(key)] = value;
        }
        return true;
    }
}
