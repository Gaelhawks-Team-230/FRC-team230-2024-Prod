#include "util/Loopcount.h"

Loopcount *Loopcount::GetInstance()
{
    if (m_loopcount == nullptr)
    {
        m_loopcount = new Loopcount();
    }
    return m_loopcount;
}

Loopcount *Loopcount::m_loopcount = nullptr;