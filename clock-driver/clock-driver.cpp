#include <cstdio>
#include <vector>
#include <functional>
#include <thread>
#include <chrono>
#include <csignal>

#include <bcm_host.h>		/* required to use bcm_host_get_peripheral_address() */
#include <fcntl.h>		/* required to use open(), close(), usleep() */
#include <sys/mman.h>		/* required to use mmap(), munmap() */

using namespace std;
using namespace std::chrono;

uint32_t* GetAddress(void* map, uint32_t offset)
{
	auto base = reinterpret_cast<uint8_t*>(map);
	auto cdst = base + offset;
	auto idst = reinterpret_cast<uint32_t*>(cdst);
	return idst;
}

int GetFSELOffset(int id)
{
	constexpr auto GPFSEL0 = 0x00;
	constexpr auto GPFSEL1 = 0x04;
	constexpr auto GPFSEL2 = 0x08;
	constexpr auto GPFSEL3 = 0x0C;
	constexpr auto GPFSEL4 = 0x10;
	constexpr auto GPFSEL5 = 0x14;
	if(0 <= id && id < 10)
		return GPFSEL0;
	else if(id < 20)
		return GPFSEL1;
	else if(id < 30)
		return GPFSEL2;
	else if(id < 40)
		return GPFSEL3;
	else if(id < 50)
		return GPFSEL4;
	else if(id < 54)
		return GPFSEL5;
	return -1;
}

void MaskedWrite(uint32_t* dst, uint32_t mask, uint32_t value)
{
	auto oldValue = *dst;
	*dst = (value & mask) | (oldValue & ~mask);
}

constexpr uint32_t CreateMask(int numBits)
{
	uint32_t ret = 0;
	for(int i = 0; i < numBits; ++i)
	{
		ret <<= 1;
		ret |= 1;
	}
	return ret;
}

class CMaskedWriter
{
public:
	explicit CMaskedWriter(int shift, int shiftUnit);
	void Write(uint32_t* dst, uint32_t value) const;
private:
	const int shiftBit;
	const uint32_t mask;
};

CMaskedWriter::CMaskedWriter(int shift, int shiftUnit)
	: shiftBit{ shiftUnit * shift }
	, mask{ CreateMask(shiftUnit) << shiftBit }
{
}

void CMaskedWriter::Write(uint32_t* dst, uint32_t value) const
{
	MaskedWrite(dst, mask, value << shiftBit);
}

void WriteFSEL(void* map, int id, uint8_t func)
{
	auto offset = GetFSELOffset(id);
	if(offset == -1)
		return;

	constexpr int numFuncBits = 3;
	CMaskedWriter writer{ id % 10, numFuncBits };
	writer.Write(GetAddress(map, offset), func);
}

uint32_t* GetSetAddress(void* map, int id)
{
	constexpr auto GPSET0 = 0x1C;
	constexpr auto GPSET1 = 0x20;
	if(0 <= id && id < 32)
	{
		return GetAddress(map, GPSET0);
	}
	else if(id < 54)
	{
		return GetAddress(map, GPSET1);
	}
	return nullptr;
}

uint32_t* GetClearAddress(void* map, int id)
{
	constexpr auto GPCLR0 = 0x28;
	constexpr auto GPCLR1 = 0x2C;
	if(0 <= id && id < 32)
	{
		return GetAddress(map, GPCLR0);
	}
	else if(id < 54)
	{
		return GetAddress(map, GPCLR1);
	}
	return nullptr;
}

uint32_t GetSetClearValue(int id)
{
	return 0b1 << (id % 32);
}

class CGPIO
{
public:
	explicit CGPIO(void* map, int id);
	CGPIO(const CGPIO&) = delete;
	CGPIO& operator =(const CGPIO&) = delete;
	CGPIO(CGPIO&&);
	CGPIO& operator =(CGPIO&&);
	~CGPIO();
	void Set();
	void Clear();
private:
	static constexpr int errID = -1;

	void* const map;
	const int id;
	uint32_t* const setAddress;
	uint32_t* const clearAddress;
	const uint32_t setClearValue;
};

CGPIO::CGPIO(void* map, int id)
	: map{ map }, id{ id }
	, setAddress { GetSetAddress(map, id) }
	, clearAddress { GetClearAddress(map, id) }
	, setClearValue { GetSetClearValue(id) }
{
	WriteFSEL(map, id, 0b001);
}

CGPIO::CGPIO(CGPIO&& a)
	: map{ a.map }, id{ a.id }
	, setAddress { a.setAddress }
	, clearAddress { a.clearAddress }
	, setClearValue { a.setClearValue }
{
	const_cast<int&>(a.id) = errID;
}

CGPIO& CGPIO::operator = (CGPIO&& a)
{
	if(&a == this)
		return *this;
	return *this = CGPIO(move(a));
}

CGPIO::~CGPIO()
{
	if(id != errID)
		WriteFSEL(map, id, 0b000);
}

void CGPIO::Set()
{
	*setAddress = setClearValue;
}

void CGPIO::Clear()
{
	*clearAddress = setClearValue;
}

class CMemFile
{
public:
	explicit CMemFile();
	int GetFD() const noexcept { return fd; }
	~CMemFile();

	struct OpenError {};
private:
	static constexpr int errFD = -1;

	const int fd;
};

CMemFile::CMemFile()
	: fd{ open("/dev/mem", (O_RDWR | O_SYNC)) }
{
	if(fd == errFD)
		throw OpenError{};
}

CMemFile::~CMemFile()
{
	close(fd);
}

class CMemMap
{
public:
	explicit CMemMap();
	void* Get() noexcept { return map; }
	~CMemMap();

	struct MapError{};
private:
	static constexpr size_t block_size = 4096;
	static void* CreateMemMap();

	void* const map;
};

void* CMemMap::CreateMemMap()
{
	constexpr off_t gpio_offset = 0x00200000;

	CMemFile memFile;
	auto adr_gpio_base = bcm_host_get_peripheral_address();
	auto map = mmap(NULL,
	            block_size,
	            PROT_WRITE,
	            MAP_SHARED,
	            memFile.GetFD(),
	            adr_gpio_base + gpio_offset);
	return map;
}

CMemMap::CMemMap()
	: map { CreateMemMap() }
{
	if(map == MAP_FAILED)
		throw MapError{};
}

CMemMap::~CMemMap()
{
	munmap(map, block_size);
}

void Pulse(CGPIO& gpio)
{
	gpio.Set();
	gpio.Clear();
}

class CShiftRegister
{
public:
	explicit CShiftRegister(CMemMap& memMap, int siID, int rckID, int sckID);
	~CShiftRegister();
	void Write(uint8_t value);
	void Flush();
private:
	CGPIO si;
	CGPIO rck;
	CGPIO sck;
};

CShiftRegister::CShiftRegister(CMemMap& memMap, int siID, int rckID, int sckID)
	: si{ memMap.Get(), siID }
	, rck{ memMap.Get(), rckID }
	, sck{ memMap.Get(), sckID }
{
}

void CShiftRegister::Write(uint8_t value)
{
	for(int i = 0; i < 8; ++i)
	{
		auto lsb = value & 0b1;
		if(lsb)
			si.Set();
		else
			si.Clear();
		Pulse(sck);
		value >>= 1;
	}
}

void CShiftRegister::Flush()
{
	Pulse(rck);
}

CShiftRegister::~CShiftRegister()
{
	Write(0b11111111);
}

uint8_t Get7SegBits(int value)
{
	switch(value)
	{
	case 0x0: return 0b00000011;
	case 0x1: return 0b10011111;
	case 0x2: return 0b00100101;
	case 0x3: return 0b00001101;
	case 0x4: return 0b10011001;
	case 0x5: return 0b01001001;
	case 0x6: return 0b01000001;
	case 0x7: return 0b00011111;
	case 0x8: return 0b00000001;
	case 0x9: return 0b00001001;
	case 0xA: return 0b00010001;
	case 0xB: return 0b11000001;
	case 0xC: return 0b01100011;
	case 0xD: return 0b10000101;
	case 0xE: return 0b01100001;
	case 0xF: return 0b01110001;
	default: return 0b11111111;
	}
}

uint8_t Get7SegBitsWithPoint(int value, bool hasPoint)
{
	return Get7SegBits(value) & (hasPoint ? 0b11111110 : 0b11111111);
}

class C4Digits
{
public:
	explicit C4Digits(CMemMap& memMap, int id1, int id2, int id3, int id4);
	void Switch(function<void(int)> write, function<void()> flush);
	~C4Digits();
private:
	static constexpr int numDigits = 4;

	void Clear();

	vector<CGPIO> digits;
	int curDigitIdx;
};

C4Digits::C4Digits(CMemMap& memMap, int id1, int id2, int id3, int id4)
	: curDigitIdx{ numDigits - 1 }
{
	digits.emplace_back(memMap.Get(), id1);
	digits.emplace_back(memMap.Get(), id2);
	digits.emplace_back(memMap.Get(), id3);
	digits.emplace_back(memMap.Get(), id4);
	Clear();
}

void C4Digits::Switch(function<void(int)> write, function<void()> flush)
{
	auto lastDigitIdx = curDigitIdx;
	curDigitIdx = (curDigitIdx + 1) % numDigits;

	write(curDigitIdx);
	digits[lastDigitIdx].Clear();
	flush();
	digits[curDigitIdx].Set();
}

void C4Digits::Clear()
{
	for(auto& x : digits)
		x.Clear();
}

C4Digits::~C4Digits()
{
	Clear();
}

int GetDigit(uint16_t value4, int digitIdx)
{
	return (value4 >> ((3 - digitIdx) * 4)) & 0b1111;
}

struct SMyValue
{
	uint16_t value4;
	bool point;
};

struct SSharedValues
{
	CShiftRegister& reg;
	const SMyValue& value;
	C4Digits& digits;
};

void DispThread(const bool* pFinished, const SSharedValues* pSharedValues)
{
	auto& finished = *pFinished;
	auto& reg = pSharedValues->reg;
	auto& value = pSharedValues->value;
	auto& digits = pSharedValues->digits;

	auto write = [&reg, &value] (int curDigitIdx)
	{
		reg.Write(Get7SegBitsWithPoint(GetDigit(value.value4, curDigitIdx), (curDigitIdx == 1 && value.point)));
	};
	auto flush = [&reg] ()
	{
		reg.Flush();
	};
	while(!finished)
	{
		digits.Switch(write, flush);
		usleep(5000);
	}
}

class CDispThread
{
public:
	CDispThread(const SSharedValues* pSharedValues);
	~CDispThread();
private:
	thread th;
	bool finished;
};

CDispThread::CDispThread(const SSharedValues* pSharedValues)
	: th{ DispThread, &finished, pSharedValues }, finished{ false }
{
}

CDispThread::~CDispThread()
{
	finished = true;
	th.join();
}

uint16_t CreateValue4(uint8_t digit12, uint8_t digit34)
{
	return (digit12 << 8) | digit34;
}

uint8_t CreateValue2(int value)
{
	return ((value / 10) << 4) | (value % 10);
}

static volatile sig_atomic_t g_finished = 0;

void SigHandler(int sig)
{
	g_finished = 1;
}

bool SetSigHandlers()
{
	if(signal(SIGINT, SigHandler) == SIG_ERR)
	{
		printf("Failed to set SIGINT handler\n");
		return false;
	}

	if(signal(SIGTERM, SigHandler) == SIG_ERR)
	{
		printf("Failed to set SIGTERM handler\n");
		return false;
	}
	return true;
}

void SetTimeZone()
{
	static char envStr[] = "TZ=Asia/Tokyo";
	putenv(envStr);
	tzset();
}

SMyValue GetMyValue()
{
	auto now = system_clock::now();
	auto nowTime = system_clock::to_time_t(now);
	auto pTime = localtime(&nowTime);
	auto hour = pTime->tm_hour;
	auto min = pTime->tm_min;
	auto sec = pTime->tm_sec;
	return SMyValue
	{
		CreateValue4(CreateValue2(hour), CreateValue2(min)),
		(sec % 2) != 0,
	};
}

int main()
{
	if(!SetSigHandlers())
		return 1;
	try
	{
		SetTimeZone();

		CMemMap memMap;
		CShiftRegister reg{ memMap, 21, 20, 16 };
		C4Digits digits{ memMap, 26, 19, 13, 6  };
		auto value = GetMyValue();
		auto sharedValues = SSharedValues{ reg, value, digits };
		CDispThread th{ &sharedValues };
		while(!g_finished)
		{
			value = GetMyValue();
			this_thread::sleep_for(milliseconds { 100 });
		}
	}
	catch(...)
	{
		try { throw; }
		catch(CMemFile::OpenError)
		{
			printf("open error\n");
		}
		catch(...)
		{
			printf("unknown error\n");
		}
		return 1;
	}
	printf("\n");
	printf("Finished successfully.\n");
}

