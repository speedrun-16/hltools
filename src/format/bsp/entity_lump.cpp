#include "entity_lump.h"

namespace format
{
    const char *entity::value(const char *key) const
    {
        for (const auto &p : pairs_)
        {
            if (p.first == key)
                return p.second.c_str();
        }
        return "";
    }

    bool entity::has(const char *key) const
    {
        for (const auto &p : pairs_)
        {
            if (p.first == key)
                return true;
        }
        return false;
    }

    void entity::set(const char *key, const char *value)
    {
        if (value[0] == '\0')
        {
            remove(key);
            return;
        }
        for (auto &p : pairs_)
        {
            if (p.first == key)
            {
                p.second = value;
                return;
            }
        }
        prepend(key, value);
    }

    void entity::remove(const char *key)
    {
        for (auto it = pairs_.begin(); it != pairs_.end(); ++it)
        {
            if (it->first == key)
            {
                pairs_.erase(it);
                return;
            }
        }
    }

    namespace
    {
        class token_reader
        {
        public:
            explicit token_reader(const std::string &text) : text_(text) {}

            bool next(std::string &out)
            {
                skip_space();
                if (pos_ >= text_.size())
                    return false;
                char c = text_[pos_];
                if (c == '{' || c == '}')
                {
                    out.assign(1, c);
                    pos_++;
                    return true;
                }
                if (c == '"')
                {
                    pos_++;
                    size_t start = pos_;
                    while (pos_ < text_.size() && text_[pos_] != '"')
                        pos_++;
                    out.assign(text_, start, pos_ - start);
                    if (pos_ < text_.size())
                        pos_++;
                    return true;
                }
                size_t start = pos_;
                while (pos_ < text_.size() && !is_space(text_[pos_]))
                    pos_++;
                out.assign(text_, start, pos_ - start);
                return true;
            }

        private:
            static bool is_space(char c) {
                return c == ' ' || c == '\t' || c == '\r' || c == '\n';
            }
            void skip_space() {
                while (pos_ < text_.size() && is_space(text_[pos_]))
                    pos_++;
            }

            const std::string &text_;
            size_t pos_ = 0;
        };
    }

    std::vector<entity> parse_entities(const std::string &text)
    {
        std::vector<entity> out;
        token_reader reader(text);
        std::string tok;
        while (reader.next(tok))
        {
            if (tok != "{")
                continue;
            entity ent;
            std::string key, value;
            while (reader.next(key) && key != "}")
            {
                if (!reader.next(value))
                    break;
                ent.prepend(key, value);
            }
            out.push_back(std::move(ent));
        }
        return out;
    }

    std::string write_entities(const std::vector<entity> &entities)
    {
        std::string out;
        for (const auto &ent : entities)
        {
            if (ent.empty())
                continue;
            out += "{\n";
            for (const auto &p : ent.pairs())
            {
                out += '"';
                out += p.first;
                out += "\" \"";
                out += p.second;
                out += "\"\n";
            }
            out += "}\n";
        }
        out.push_back('\0');
        return out;
    }
}
