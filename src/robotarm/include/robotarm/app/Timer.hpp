/**
 * @file Timer.hpp
 * @brief Timer
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#ifndef TIMER_H_
#define TIMER_H_

#include <chrono>

namespace utility
{
    /**
     * @brief Timer keeps track of time intervals
     * @tparam T Time unit
     */
	template<typename T>
	class Timer
	{
	public:
	    Timer(T interval = (T)0)
		    : interval(interval), previous(std::chrono::steady_clock::now() - interval)
		{
	    }

        /**
         * @brief Check time passed since last mark
         *
         * @return Time passed
         */
	    auto operator()() const
		{
	        return std::chrono::duration_cast<T>(std::chrono::steady_clock::now() - previous).count();
	    }

        /**
         * @brief Check time passed since last mark
         *
         * @return Time passed
         */
	    auto Time() const
	    {
	        return (*this)();
	    }

        /**
         * @brief Reset the timer
         */
		void Reset()
		{
	        previous = std::chrono::steady_clock::now();
	    }

        /**
         * @brief Check if interval is completed
         *
         * @return `true` if interval completed, else `false`
         */
	    bool Finished() const
	    {
	        return std::chrono::duration_cast<T>(std::chrono::steady_clock::now() - previous) >= interval;
	    }

        /**
         * @brief Set new interval
         *
         * @param interval New interval
         */
        void setInterval(T interval)
        {
            this->interval = interval;
        }

	private:
	    T interval; ///< Interval time
	    std::chrono::time_point<std::chrono::steady_clock> previous; ///< The previous mark
	};

	typedef Timer<std::chrono::seconds> SecTimer;
	typedef Timer<std::chrono::milliseconds> MilliTimer;
	typedef Timer<std::chrono::nanoseconds> NanoTimer;
} // namespace utility

#endif // TIMER_H_
