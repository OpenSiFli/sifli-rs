/// Clock domain marker types.
///
/// These are zero-sized types used as `RccGetFreq::Clock` associated types
/// to identify which clock domain a peripheral belongs to.
// TODO: once peripheral drivers start borrowing `&'d Token` at construction,
// these types can gain `frequency()` methods and be collected into a
// `ClockControl` struct returned by `init()`, enabling compile-time
// clock-safety guarantees.

macro_rules! define_clock_token {
    ($name:ident) => {
        pub struct $name(());
    };
}

define_clock_token!(Hclk);
define_clock_token!(Pclk);
define_clock_token!(Pclk2);
define_clock_token!(ClkPeri);
define_clock_token!(ClkPeriDiv2);
define_clock_token!(ClkUsb);
define_clock_token!(ClkWdt);
define_clock_token!(ClkRtc);
define_clock_token!(ClkMpi1);
define_clock_token!(ClkMpi2);
define_clock_token!(ClkAudPll);
define_clock_token!(ClkAudPllDiv16);
define_clock_token!(LpHclk);
define_clock_token!(LpMacClk);
