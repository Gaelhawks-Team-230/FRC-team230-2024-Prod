#pragma once

class Loopcount
{
public:
    static Loopcount *GetInstance();

    unsigned int GetLoopCount() { return m_count; }
    void IncrementLoopCount() { m_count++; }
    void ResetLoopCount() { m_count = 0; }

private:
    Loopcount() { m_count = 0; }
    static Loopcount *m_loopcount;
    unsigned int m_count;
};

