#pragma once

class Loopcount
{
public:
    static Loopcount *GetInstance()
    {
        if (m_loopcount == nullptr)
        {
            m_loopcount = new Loopcount();
        }
        return m_loopcount;
    }

    unsigned int GetLoopCount() { return m_count; }
    void IncrementLoopCount() { m_count++; }
    void ResetLoopCount() { m_count = 0; }

private:
    Loopcount() { m_count = 0; }
    static Loopcount *m_loopcount;
    unsigned int m_count;
};

Loopcount *Loopcount::m_loopcount = nullptr;