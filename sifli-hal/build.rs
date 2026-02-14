use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::path::PathBuf;
use std::process::Command;

use proc_macro2::TokenStream;
use quote::format_ident;
use quote::quote;

mod build_serde;
// Structures imported from build_serde.rs
use build_serde::{IR, FieldSet, Field, Interrupts, Peripherals};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Retrieve all enabled features
    let chip_name = match env::vars()
        .map(|(a, _)| a)
        // Only keep chip features like CARGO_FEATURE_SF32LB52X, ignore *_LCPU helper features.
        .filter(|x| x.starts_with("CARGO_FEATURE_SF32") && !x.ends_with("_LCPU"))
        .get_one()
    {
        Ok(x) => x,
        Err(GetOneError::None) => panic!("No sf32xx Cargo feature enabled"),
        Err(GetOneError::Multiple) => panic!("Multiple sf32xx Cargo features enabled"),
    }
        .strip_prefix("CARGO_FEATURE_")
        .unwrap()
        .to_ascii_lowercase();

    let _time_driver_peripheral = match env::vars()
        .map(|(key, _)| key)
        .filter(|x| x.starts_with("CARGO_FEATURE_TIME_DRIVER_"))
        .get_one()
    {
        Ok(x) => Some(
            x.strip_prefix("CARGO_FEATURE_TIME_DRIVER_")
                .unwrap()
                .to_ascii_uppercase()
        ),
        Err(GetOneError::None) => None,
        Err(GetOneError::Multiple) => panic!("Multiple time-driver-xx Cargo features enabled"),
    };

    println!("cargo:rerun-if-changed=data/{}", chip_name);
    let data_dir = Path::new("data").join(&chip_name);

    // Read and parse HPSYS_RCC.yaml
    let rcc_path = data_dir.join("HPSYS_RCC.yaml");
    let rcc_content = fs::read_to_string(&rcc_path).unwrap();
    
    let ir: IR = serde_yaml::from_str(&rcc_content)
        .map_err(|e| format!("Failed to parse HPSYS_RCC.yaml: {}", e))?;

    let _blocks = ir.blocks;
    let fieldsets = ir.fieldsets;

    // Read and parse LPSYS_RCC.yaml (optional, for LCPU peripheral support)
    let lpsys_rcc_path = data_dir.join("LPSYS_RCC.yaml");
    let lpsys_fieldsets = if lpsys_rcc_path.exists() {
        let lpsys_rcc_content = fs::read_to_string(&lpsys_rcc_path)
            .map_err(|e| format!("Failed to read LPSYS_RCC.yaml: {}", e))?;
        let lpsys_ir: IR = serde_yaml::from_str(&lpsys_rcc_content)
            .map_err(|e| format!("Failed to parse LPSYS_RCC.yaml: {}", e))?;
        Some(lpsys_ir.fieldsets)
    } else {
        None
    };

    // Read and parse interrupts.yaml
    let interrupts_path = data_dir.join("interrupts.yaml");
    let interrupts_content = fs::read_to_string(&interrupts_path)
        .map_err(|e| format!("Failed to read interrupts.yaml: {}", e))?;
    
    let interrupts: Interrupts = serde_yaml::from_str(&interrupts_content)
        .map_err(|e| format!("Failed to parse interrupts.yaml: {}", e))?;

    // Read and parse peripherals.yaml
    let peripherals_path = data_dir.join("peripherals.yaml");
    let peripherals_content = fs::read_to_string(&peripherals_path)
        .map_err(|e| format!("Failed to read peripherals.yaml: {}", e))?;
    
    let peripherals: Peripherals = serde_yaml::from_str(&peripherals_content)
        .map_err(|e| format!("Failed to parse peripherals.yaml: {}", e))?;

    // Read and parse HPSYS_CFG.yaml
    let cfg_path = data_dir.join("HPSYS_CFG.yaml");
    let cfg_content = fs::read_to_string(&cfg_path)?;
    let cfg_ir: IR = serde_yaml::from_str(&cfg_content)?;

    // Read and parse pinmux_signals.yaml
    let pinmux_signals_path = data_dir.join("pinmux_signals.yaml");
    let pinmux_signals_content = fs::read_to_string(&pinmux_signals_path)?;
    let pinmux_signals: build_serde::PinmuxSignals = serde_yaml::from_str(&pinmux_signals_content)?;

    // Read and parse pinmux.yaml
    let pinmux_path = data_dir.join("pinmux.yaml");
    let pinmux_content = fs::read_to_string(&pinmux_path)?;
    let pinmux: build_serde::Pinmux = serde_yaml::from_str(&pinmux_content)?;

    // Read and parse adc.yaml
    let adc_path = data_dir.join("adc.yaml");
    let adc_content = fs::read_to_string(&adc_path)?;
    let adc: build_serde::Adc = serde_yaml::from_str(&adc_content)?;

    // Read and parse dma.yaml
    let dma_path = data_dir.join("dma.yaml");
    let dma_content = fs::read_to_string(&dma_path)?;
    let dma: build_serde::Dma = serde_yaml::from_str(&dma_content)
        .map_err(|e| format!("Failed to parse dma.yaml: {}", e))?;

    // Read and parse mailbox.yaml
    let mailbox_path = data_dir.join("mailbox.yaml");
    let mailbox_content = fs::read_to_string(&mailbox_path)?;
    let mailbox: build_serde::Mailbox = serde_yaml::from_str(&mailbox_content)
        .map_err(|e| format!("Failed to parse mailbox.yaml: {}", e))?;
    
    // Get output path from env
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let dest_path = out_dir.join("_generated.rs");

    let mut token_stream = TokenStream::new();

    token_stream.extend(quote! {
        use crate::peripherals::*;
        use crate::gpio::Pin;
        use crate::gpio::SealedPin;
    });

    // Generate interrupt mod
    let interrupt_mod = generate_interrupt_mod(&interrupts);
    token_stream.extend(interrupt_mod);

    // Generate peripherals singleton
    let peripherals_singleton = generate_peripherals_singleton(&peripherals, &dma, &mailbox);
    token_stream.extend(peripherals_singleton);

    // Generate rcc implementations
    let rcc_impls = generate_rcc_impls(&peripherals, &fieldsets, &lpsys_fieldsets);
    token_stream.extend(rcc_impls);

    // Generate pin implementations
    let pin_impls = generate_pin_impls(&pinmux, &pinmux_signals, &cfg_ir.fieldsets);
    token_stream.extend(pin_impls);

    // Generate ADC implementations
    let adc_impls = generate_adc_impls(&adc);
    token_stream.extend(adc_impls);

    // Generate DMA implementations
    let dma_impls = generate_dma_impls(&dma);
    token_stream.extend(dma_impls);

    // Write main HAL codegen output to file.
    let mut file = File::create(&dest_path).unwrap();
    write!(file, "{}", token_stream).unwrap();
    rustfmt(&dest_path);

    Ok(())
}

fn generate_rcc_impls(
    peripherals: &Peripherals,
    fieldsets: &BTreeMap<String, FieldSet>,
    lpsys_fieldsets: &Option<BTreeMap<String, FieldSet>>,
) -> TokenStream {
    let mut implementations = TokenStream::new();

    // Get HPSYS RCC register fieldsets
    let rstr1 = fieldsets.get("RSTR1").expect("RSTR1 fieldset not found");
    let rstr2 = fieldsets.get("RSTR2").expect("RSTR2 fieldset not found");
    let enr1 = fieldsets.get("ENR1").expect("ENR1 fieldset not found");
    let enr2 = fieldsets.get("ENR2").expect("ENR2 fieldset not found");

    // Generate HCPU peripheral RCC implementations
    for peripheral in &peripherals.hcpu {
        if !peripheral.enable_reset {
            continue;
        }

        // TODO: Is there a better way to handle this?
        if peripheral.ignore_missing_enable_reset {
            let peripheral_name_ident = format_ident!("{}", peripheral.name);
            let impl_tokens = quote! {
                impl crate::rcc::SealedRccEnableReset for #peripheral_name_ident {}
                impl crate::rcc::RccEnableReset for #peripheral_name_ident {}
            };
            implementations.extend(impl_tokens);
            continue;
        }
        // Get field name (prefer rcc_field if available)
        let field_name = &peripheral.rcc_field.clone()
            .unwrap_or(peripheral.name.clone()).to_lowercase();

        // Find matching fields in RCC registers
        let (enr_reg, _enr_field) = find_field_in_registers(&[
            ("ENR1", enr1),
            ("ENR2", enr2),
        ], field_name).unwrap_or_else(|| panic!("No ENR field found for peripheral {}", peripheral.name));

        let (rstr_reg, _rstr_field) = find_field_in_registers(&[
            ("RSTR1", rstr1),
            ("RSTR2", rstr2),
        ], field_name).unwrap_or_else(|| panic!("No RSTR field found for peripheral {}", peripheral.name));
        let field_set_ident = format_ident!("set_{}", field_name);
        let field_name_ident = format_ident!("{}", field_name);
        let enr_reg_ident = format_ident!("{}", enr_reg.to_lowercase());
        let rstr_reg_ident = format_ident!("{}", rstr_reg.to_lowercase());

        let peripheral_name_ident = format_ident!("{}", peripheral.name);
        let impl_tokens = quote! {
            impl crate::rcc::SealedRccEnableReset for #peripheral_name_ident {
                #[inline(always)]
                fn rcc_enable() {
                    crate::pac::HPSYS_RCC.#enr_reg_ident().modify(|w| w.#field_set_ident(true));
                }

                #[inline(always)]
                fn rcc_disable() {
                    crate::pac::HPSYS_RCC.#enr_reg_ident().modify(|w| w.#field_set_ident(false));
                }

                #[inline(always)]
                fn rcc_reset() {
                    crate::pac::HPSYS_RCC.#rstr_reg_ident().modify(|w| w.#field_set_ident(true));
                    while !crate::pac::HPSYS_RCC.#rstr_reg_ident().read().#field_name_ident() {};
                    crate::pac::HPSYS_RCC.#rstr_reg_ident().modify(|w| w.#field_set_ident(false));
                }
            }
            impl crate::rcc::RccEnableReset for #peripheral_name_ident {}
        };
        implementations.extend(impl_tokens);
    }

    // Generate LCPU peripheral RCC implementations (using LPSYS_RCC)
    if let Some(lp_fieldsets) = lpsys_fieldsets {
        let lp_rstr1 = lp_fieldsets.get("RSTR1").expect("LPSYS RSTR1 fieldset not found");
        let lp_enr1 = lp_fieldsets.get("ENR1").expect("LPSYS ENR1 fieldset not found");

        for peripheral in &peripherals.lcpu {
            if !peripheral.enable_reset {
                continue;
            }

            if peripheral.ignore_missing_enable_reset {
                let peripheral_name_ident = format_ident!("{}", peripheral.name);
                let impl_tokens = quote! {
                    impl crate::rcc::SealedRccEnableReset for #peripheral_name_ident {}
                    impl crate::rcc::RccEnableReset for #peripheral_name_ident {}
                };
                implementations.extend(impl_tokens);
                continue;
            }

            // Get field name (prefer rcc_field if available)
            let field_name = &peripheral.rcc_field.clone()
                .unwrap_or(peripheral.name.clone()).to_lowercase();

            // Find matching fields in LPSYS RCC registers
            let enr_field = find_field_in_registers(&[
                ("ENR1", lp_enr1),
            ], field_name);

            let rstr_field = find_field_in_registers(&[
                ("RSTR1", lp_rstr1),
            ], field_name);

            // Both enable and reset fields must exist for full implementation
            match (enr_field, rstr_field) {
                (Some((enr_reg, _)), Some((rstr_reg, _))) => {
                    let field_set_ident = format_ident!("set_{}", field_name);
                    let field_name_ident = format_ident!("{}", field_name);
                    let enr_reg_ident = format_ident!("{}", enr_reg.to_lowercase());
                    let rstr_reg_ident = format_ident!("{}", rstr_reg.to_lowercase());

                    let peripheral_name_ident = format_ident!("{}", peripheral.name);
                    let impl_tokens = quote! {
                        impl crate::rcc::SealedRccEnableReset for #peripheral_name_ident {
                            #[inline(always)]
                            fn rcc_enable() {
                                crate::pac::LPSYS_RCC.#enr_reg_ident().modify(|w| w.#field_set_ident(true));
                            }

                            #[inline(always)]
                            fn rcc_disable() {
                                crate::pac::LPSYS_RCC.#enr_reg_ident().modify(|w| w.#field_set_ident(false));
                            }

                            #[inline(always)]
                            fn rcc_reset() {
                                crate::pac::LPSYS_RCC.#rstr_reg_ident().modify(|w| w.#field_set_ident(true));
                                while !crate::pac::LPSYS_RCC.#rstr_reg_ident().read().#field_name_ident() {};
                                crate::pac::LPSYS_RCC.#rstr_reg_ident().modify(|w| w.#field_set_ident(false));
                            }
                        }
                        impl crate::rcc::RccEnableReset for #peripheral_name_ident {}
                    };
                    implementations.extend(impl_tokens);
                }
                _ => {
                    // Generate empty impl if fields are missing (some peripherals may only have reset or enable)
                    let peripheral_name_ident = format_ident!("{}", peripheral.name);
                    let impl_tokens = quote! {
                        impl crate::rcc::SealedRccEnableReset for #peripheral_name_ident {}
                        impl crate::rcc::RccEnableReset for #peripheral_name_ident {}
                    };
                    implementations.extend(impl_tokens);
                }
            }
        }
    }

    implementations.extend(quote! {use crate::time::Hertz;});

    // Generate RccGetFreq for HCPU peripherals
    for peripheral in &peripherals.hcpu {
        if let Some(clock) = peripheral.clock.clone() {
            let clock_name_ident = format_ident!("{}", clock);
            let clock_token_type = clock_to_token_type(&clock);
            let peripheral_name_ident = format_ident!("{}", peripheral.name);
            let impl_tokens = quote! {
                impl crate::rcc::SealedRccGetFreq for #peripheral_name_ident {
                    fn get_freq() -> Option<Hertz> {
                        crate::rcc::clocks().#clock_name_ident.into()
                    }
                }
                impl crate::rcc::RccGetFreq for #peripheral_name_ident {
                    type Clock = #clock_token_type;
                }
            };

            implementations.extend(impl_tokens);
        }
    }

    // Generate RccGetFreq for LCPU peripherals
    // LPSYS clocks read directly from hardware (not from cached CLOCK_FREQS),
    // because LCPU may change LPSYS clocks independently.
    for peripheral in &peripherals.lcpu {
        if let Some(clock) = peripheral.clock.clone() {
            let clock_token_type = clock_to_token_type(&clock);
            let peripheral_name_ident = format_ident!("{}", peripheral.name);
            let freq_expr = lpsys_clock_to_hw_read(&clock);
            let impl_tokens = quote! {
                impl crate::rcc::SealedRccGetFreq for #peripheral_name_ident {
                    fn get_freq() -> Option<Hertz> {
                        #freq_expr
                    }
                }
                impl crate::rcc::RccGetFreq for #peripheral_name_ident {
                    type Clock = #clock_token_type;
                }
            };

            implementations.extend(impl_tokens);
        }
    }

    implementations
}

fn lpsys_clock_to_hw_read(clock: &str) -> TokenStream {
    match clock {
        "lp_hclk" => quote! { crate::rcc::get_lpsys_hclk_freq() },
        "lp_mac_clk" => quote! { crate::rcc::get_lpsys_mac_clk_freq() },
        _ => panic!("Unknown LPSYS clock domain: {}", clock),
    }
}

fn clock_to_token_type(clock: &str) -> TokenStream {
    match clock {
        "hclk" => quote! { crate::rcc::Hclk },
        "pclk" => quote! { crate::rcc::Pclk },
        "pclk2" => quote! { crate::rcc::Pclk2 },
        "clk_peri" => quote! { crate::rcc::ClkPeri },
        "clk_peri_div2" => quote! { crate::rcc::ClkPeriDiv2 },
        "clk_usb" => quote! { crate::rcc::ClkUsb },
        "clk_wdt" => quote! { crate::rcc::ClkWdt },
        "clk_rtc" => quote! { crate::rcc::ClkRtc },
        "clk_mpi1" => quote! { crate::rcc::ClkMpi1 },
        "clk_mpi2" => quote! { crate::rcc::ClkMpi2 },
        "clk_aud_pll" => quote! { crate::rcc::ClkAudPll },
        "clk_aud_pll_div16" => quote! { crate::rcc::ClkAudPllDiv16 },
        "lp_hclk" => quote! { crate::rcc::LpHclk },
        "lp_mac_clk" => quote! { crate::rcc::LpMacClk },
        _ => panic!("Unknown clock domain: {}", clock),
    }
}

fn find_field_in_registers<'a>(
    registers: &[(&str, &'a FieldSet)],
    field_name: &str,
) -> Option<(String, &'a Field)> {
    for (reg_name, fieldset) in registers {
        if let Some(field) = fieldset.fields.iter().find(|f| f.name.to_lowercase() == field_name) {
            return Some((reg_name.to_string(), field));
        }
    }
    None
}

fn generate_interrupt_mod(interrupts: &Interrupts) -> TokenStream {
    let interrupt_names: Vec<_> = interrupts.hcpu
        .iter()
        .map(|int| {
            let name = &int.name;
            quote::format_ident!("{}", name)
        })
        .collect();
    quote! {
        embassy_hal_internal::interrupt_mod!(
            #(#interrupt_names),*
        );
    }
}

fn generate_peripherals_singleton(
    peripherals: &Peripherals,
    dma: &build_serde::Dma,
    mailbox: &build_serde::Mailbox,
) -> TokenStream {
    // Generate singletons for HCPU peripherals
    let hcpu_peripheral_names: Vec<_> = peripherals.hcpu
        .iter()
        .map(|p| {
            let name = &p.name;
            quote::format_ident!("{}", name)
        })
        .collect();

    // Generate singletons for LCPU peripherals
    let lcpu_peripheral_names: Vec<_> = peripherals.lcpu
        .iter()
        .map(|p| {
            let name = &p.name;
            quote::format_ident!("{}", name)
        })
        .collect();

    // TODO: move pin num to chip info
    let gpio_pins: Vec<_> = (0..=44)
        .map(|i| {
            let pin_name = format!("PA{}", i);
            quote::format_ident!("{}", pin_name)
        })
        .collect();

    // Iterate over all HCPU DMA controllers (e.g., DMAC1)
    let hcpu_dmac_channels: Vec<_> = dma.hcpu.controllers.iter().flat_map(|(name, controller)| {
        (1..=controller.channel_total).map(move |i| {
            let channel_name = format!("{}_CH{}", name, i);
            quote::format_ident!("{}", channel_name)
        })
    }).collect();

    // Iterate over all LCPU DMA controllers (e.g., DMAC2) if present
    let lcpu_dmac_channels: Vec<_> = dma.lcpu.as_ref()
        .map(|lcpu| {
            lcpu.controllers.iter().flat_map(|(name, controller)| {
                (1..=controller.channel_total).map(move |i| {
                    let channel_name = format!("{}_CH{}", name, i);
                    quote::format_ident!("{}", channel_name)
                })
            }).collect()
        })
        .unwrap_or_default();

    // Iterate over all Mailbox peripherals (e.g., MAILBOX1, MAILBOX2) found in the yaml.
    let mailbox_channels: Vec<_> = mailbox.iter().flat_map(|(name, config)| {
        (1..=config.channel_total).map(move |i| {
            // Generate singletons like `MAILBOX1_CH1`, `MAILBOX2_CH1`...
            let channel_name = format!("{}_CH{}", name, i);
            quote::format_ident!("{}", channel_name)
        })
    }).collect();

    quote! {
        embassy_hal_internal::peripherals! {
            #(#hcpu_peripheral_names,)*
            #(#lcpu_peripheral_names,)*
            #(#gpio_pins,)*
            #(#hcpu_dmac_channels,)*
            #(#lcpu_dmac_channels,)*
            #(#mailbox_channels,)*
        }
    }
}

fn generate_pin_impls(
    pinmux: &build_serde::Pinmux,
    pinmux_signals: &build_serde::PinmuxSignals,
    fieldsets: &BTreeMap<String, FieldSet>,
) -> TokenStream {
    let mut implementations = TokenStream::new();

    for pin in &pinmux.hcpu {
        let pin_name = pin.pin.replace("GPIO_", "P");
        
        for func in &pin.functions {
            // Try to match function against pinmux_signals
            if let Some(signal_def) = find_matching_signal(&func.function, &pinmux_signals.hcpu) {
                generate_signal_impls(
                    &mut implementations,
                    signal_def,
                    &pin_name,
                    func,
                    pinmux_signals,
                    fieldsets,
                );
            }
        }
    }
    
    implementations
}

fn generate_signal_impls(
    implementations: &mut TokenStream,
    signal_def: &build_serde::SignalDefinition,
    pin_name: &str,
    func: &build_serde::PinFunction,
    pinmux_signals: &build_serde::PinmuxSignals,
    fieldsets: &BTreeMap<String, FieldSet>,
) {
    let pin_ident = format_ident!("{}", pin_name);
    
    match &signal_def.r#type {
        build_serde::SignalType::Gpio => {
            generate_signal_gpio_impls(implementations, pin_name, &pin_ident);
        },
        build_serde::SignalType::Superimposed => {
            // For superimposed type, process each sub-signal
            for signal_name in &signal_def.signals {
                // Find the corresponding signal definition
                if let Some(sub_signal) = find_matching_signal(signal_name, &pinmux_signals.hcpu){
                    // Recursively process the sub-signal
                    generate_signal_impls(
                        implementations,
                        sub_signal,
                        pin_name,
                        func,
                        pinmux_signals,
                        fieldsets,
                    );
                }
            }
        },
        build_serde::SignalType::PeripheralMux => {
            generate_signal_peripheral_mux_impls(
                implementations,
                signal_def,
                &pin_ident,
                func,
                fieldsets,
            );
        },
        build_serde::SignalType::PeripheralNomux => {
            generate_signal_peripheral_nomux_impls(
                implementations,
                signal_def,
                &pin_ident,
                func,
            );
        },
    }
}

fn generate_signal_gpio_impls(
    implementations: &mut TokenStream,
    pin_name: &str,
    pin_ident: &proc_macro2::Ident,
) { 
    let pin_num = pin_name[2..].parse::<u8>().unwrap();
    let bank = 0u8; // For GPIO_Ax, bank is always 0
    implementations.extend(quote! {
        impl_pin!(#pin_ident, #bank, #pin_num);
    });
}

fn generate_signal_peripheral_mux_impls(
    implementations: &mut TokenStream,
    signal_def: &build_serde::SignalDefinition,
    pin_ident: &proc_macro2::Ident,
    func: &build_serde::PinFunction,
    fieldsets: &BTreeMap<String, FieldSet>,
) {
    let signal = signal_def.name.clone();
    // Handle peripheral mux implementations
    let pattern = regex::Regex::new(&format!(
        r"^{}(\d+)?_PINR(\d+)?$", signal
    )).unwrap();

    for (name, fieldset) in fieldsets.iter() {
        if let Some(captures) = pattern.captures(name) {
            let peripheral = if let Some(num) = captures.get(1) {
                format!("{}{}", signal, num.as_str())
            } else {
                signal.to_string()
            };

            for field in &fieldset.fields {
                if field.name.ends_with("_PIN") {
                    // Generate cfg_pin type name from field name
                    // Examples:
                    //   TXD_PIN -> TxdPin (USART)
                    //   CH1_PIN -> Ch1 (Timer, special case)
                    let name = field.name.replace("_PIN", "").to_lowercase();
                    
                    // Special case for Timer channels: CH1 -> Ch1 (not Ch1Pin)
                    // This keeps consistency with embassy-stm32
                    let cfg_pin = if signal.starts_with("GPTIM") || signal.starts_with("ATIM") || signal.starts_with("LPTIM") {
                        // Timer: "ch1" -> "Ch1", "ch2" -> "Ch2"
                        format!("{}{}", 
                            name.chars().next().unwrap_or_default().to_uppercase(), 
                            &name[1..])
                    } else {
                        // Other peripherals: "txd" -> "TxdPin"
                        format!("{}Pin", 
                            name.chars().next().unwrap_or_default().to_uppercase().to_string() + 
                            &name[1..])
                    };
                    
                    let trait_path_str = signal_def.pin_trait.clone().unwrap()
                        .replace("$peripheral", &peripheral)
                        .replace("$cfg_pin", &cfg_pin);
                    let trait_path = syn::parse_str::<syn::Path>(&trait_path_str)
                        .unwrap_or_else(|_| panic!("Invalid trait path: {}", trait_path_str));

                    let reg_name = format_ident!("{}_pinr", peripheral.to_lowercase());
                    let set_field = format_ident!("set_{}", field.name.to_lowercase());
                    let func_value = func.value;

                    // Determine if this is a Timer signal (special handling for PINR)
                    let is_timer = signal.starts_with("GPTIM") || signal.starts_with("ATIM") || signal.starts_with("LPTIM");
                    
                    let set_cfg_pin_impl = if is_timer {
                        // Timer PINR: Use pin number (0-44)
                        quote! {
                            crate::pac::HPSYS_CFG.#reg_name().modify(|w| 
                                w.#set_field(self.pin() as _)
                            );
                        }
                    } else {
                        // Other peripherals: Use pin_bank()
                        quote! {
                            crate::pac::HPSYS_CFG.#reg_name().modify(|w| 
                                w.#set_field(self.pin_bank() as _)
                            );
                        }
                    };
                    
                    implementations.extend(quote! {
                        impl #trait_path for #pin_ident {
                            fn fsel(&self) -> u8 {
                                #func_value
                            }

                            fn set_cfg_pin(&self) {
                                #set_cfg_pin_impl
                            }
                        }
                    });
                }
            }
        }
        
    }
}

fn generate_signal_peripheral_nomux_impls(
    implementations: &mut TokenStream,
    signal_def: &build_serde::SignalDefinition,
    pin_ident: &proc_macro2::Ident,
    func: &build_serde::PinFunction,
) {
    if let Some(pin_trait) = &signal_def.pin_trait {
        // Extract peripheral from signal name
        let peripheral = func.function.split("_").next().unwrap();

        // Get signal suffix (LCDC1_SPI_RSTB -> RSTB) (laststr)
        // and convert to PascalCase (RSTB -> Rstb)
        let raw_signal_suffix = func.function.split('_').last().unwrap_or("").to_lowercase();
        let laststr_pascal = if !raw_signal_suffix.is_empty() {
            let mut chars = raw_signal_suffix.chars();
            match chars.next() {
                Some(first) => first.to_ascii_uppercase().to_string() + chars.as_str(),
                None => String::new(),
            }
        } else {
            String::new()
        };

        let trait_path_str = pin_trait
            .replace("$peripheral", peripheral)
            .replace("$laststr", &laststr_pascal);

        let trait_path = syn::parse_str::<syn::Path>(&trait_path_str)
            .unwrap_or_else(|_| panic!("Invalid trait path: {}", trait_path_str));
        let func_value = func.value;

        implementations.extend(quote! {
            impl #trait_path for #pin_ident {
                fn fsel(&self) -> u8 {
                    #func_value
                }
            }
        });
    } else {
        panic!("No pin trait specified for peripheral nomux signal {}", signal_def.name);
    }
}

fn find_matching_signal<'a>(
    function: &str,
    signals: &'a [build_serde::SignalDefinition],
) -> Option<&'a build_serde::SignalDefinition> {
    signals.iter().find(|&signal| regex::Regex::new(&signal.name)
            .unwrap()
            .is_match(function)).map(|v| v as _)
}

fn generate_adc_impls(adc: &build_serde::Adc) -> TokenStream {
    let mut implementations = TokenStream::new();

    for adc_peri in &adc.hcpu {
        let vbat_channel_id = adc_peri.vbat_channel_id;
        let first_channel_pin = adc_peri.first_channel_pin;
        let vol_offset = adc_peri.vol_offset;
        let vol_ratio = adc_peri.vol_ratio;

        implementations.extend(quote! {
            pub const VBAT_CHANNEL_ID: u8 = #vbat_channel_id;
            pub const FIRST_CHANNEL_PIN: u8 = #first_channel_pin;
            pub const VOL_OFFSET: u16 = #vol_offset;
            pub const VOL_RATIO: u16 = #vol_ratio;
        });

        for pin_name_str in &adc_peri.pins {
            let pin_ident = format_ident!("{}", pin_name_str);
            implementations.extend(quote! {
                impl crate::adc::AdcPin for peripherals::#pin_ident {}
            });
        }
    }

    implementations
}

/// Generates DMA `Request` enum and trait implementations.
fn generate_dma_impls(dma: &build_serde::Dma) -> TokenStream {
    let mut implementations = TokenStream::new();
    let dma_hcpu = &dma.hcpu;

    // 1. Generate the `Request` enum from all dma requests across all controllers.
    let enum_variants = dma_hcpu.controllers.values().flat_map(|controller| &controller.request).map(|req| {
        let name = format_ident!("{}", to_pascal_case(&req.name));
        let id = req.id;
        quote! {
            #name = #id,
        }
    });

    implementations.extend(quote! {
        /// DMA request sources.
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[repr(u8)]
        pub enum Request {
            #(#enum_variants)*
        }
    });

    // 2. Generate `dma_trait_impl!` calls for requests where `used` is true..
    let trait_impls = dma_hcpu.controllers.values()
        .flat_map(|controller| &controller.request) // Iterate over all requests
        .filter(|req| req.used)
        .flat_map(move |req| {
            // For each request, get the necessary info
            let (peripheral_str, signal_str) = req.name.split_once('_').unwrap_or((&req.name, ""));

            // Prefer explicit module name from yaml, otherwise infer from peripheral name.
            let module_name = req.module.clone()
                .unwrap_or_else(|| {
                    // Remove trailing digits (e.g., "USART1" -> "usart")
                    peripheral_str.trim_end_matches(|c: char| c.is_ascii_digit()).to_string()
                });

            let module_ident = format_ident!("{}", module_name);
            let peripheral_ident = format_ident!("{}", peripheral_str.to_uppercase());
            let trait_ident = if signal_str.is_empty() {
                format_ident!("Dma")
            } else {
                format_ident!("{}Dma", to_pascal_case(signal_str))
            };
            let request_variant = format_ident!("{}", to_pascal_case(&req.name));

            // Now, for this single request, iterate over ALL available channels from ALL controllers
            dma_hcpu.controllers.iter().flat_map(move |(dmac_name, controller)| {
                // Clone the variables that will be moved into the inner closure.
                // This ensures each iteration of the inner map gets its own copy.
                let module_ident = module_ident.clone();
                let peripheral_ident = peripheral_ident.clone();
                let trait_ident = trait_ident.clone();
                let request_variant = request_variant.clone();

                (1..=controller.channel_total).map(move |i| {
                    // Generate an impl for each channel
                    let channel_ident = format_ident!("{}_CH{}", dmac_name, i);

                    quote! {
                        dma_trait_impl!(crate::#module_ident::#trait_ident, #peripheral_ident, #channel_ident, Request::#request_variant);
                    }
                })
            })
        });

    implementations.extend(quote! {
        #(#trait_impls)*
    });

    // 3. Generate channel constants
    // Channel ID encoding:
    //   - DMAC1: 0x00-0x07 (bit7=0)
    //   - DMAC2: 0x80-0x87 (bit7=1)
    let dmac1_count = dma_hcpu.controllers.get("DMAC1")
        .map(|c| c.channel_total as usize)
        .unwrap_or(0);
    let dmac2_count = dma.lcpu.as_ref()
        .and_then(|lcpu| lcpu.controllers.get("DMAC2"))
        .map(|c| c.channel_total as usize)
        .unwrap_or(0);
    let total_channel_count = dmac1_count + dmac2_count;

    implementations.extend(quote! {
        /// The number of channels in DMAC1.
        pub const DMAC1_CHANNEL_COUNT: usize = #dmac1_count;
        /// The number of channels in DMAC2.
        pub const DMAC2_CHANNEL_COUNT: usize = #dmac2_count;
        /// Total number of DMA channels across all controllers.
        pub const CHANNEL_COUNT: usize = #total_channel_count;
        /// Bit mask to identify DMAC2 channels (bit 7 set).
        pub const DMAC2_ID_FLAG: u8 = 0x80;
    });

    // 4. Generate DMAC1 channel implementations and interrupts
    for (dmac_name, controller) in &dma_hcpu.controllers {
        for i in 1..=controller.channel_total {
            let channel_index = i - 1;
            let channel_name_str = format!("{}_CH{}", dmac_name, i);
            let channel_ident = format_ident!("{}", channel_name_str);

            implementations.extend(quote! {
                dma_channel_impl!(#channel_ident, #channel_index);

                #[cfg(feature = "rt")]
                #[crate::interrupt]
                unsafe fn #channel_ident() {
                    <crate::peripherals::#channel_ident as crate::dma::ChannelInterrupt>::on_irq();
                }
            });
        }
    }

    // 5. Generate DMAC2 channel implementations (LCPU, optional)
    if let Some(lcpu) = &dma.lcpu {
        for (dmac_name, controller) in &lcpu.controllers {
            for i in 1..=controller.channel_total {
                // DMAC2 channels use IDs 0x80-0x87 (bit7 set to distinguish from DMAC1)
                let channel_index = 0x80u8 + (i - 1);
                let channel_name_str = format!("{}_CH{}", dmac_name, i);
                let channel_ident = format_ident!("{}", channel_name_str);

                implementations.extend(quote! {
                    dma_channel_impl!(#channel_ident, #channel_index);

                    // Note: DMAC2 interrupts are LCPU-side, not handled here
                });
            }
        }
    }

    // 6. Generate the function to enable DMA channel interrupts.
    let mut match_arms = TokenStream::new();
    let mut current_channel_id: u8 = 0;

    for (dmac_name, controller) in &dma_hcpu.controllers {
        for i in 1..=controller.channel_total {
            let channel_name_str = format!("{}_CH{}", dmac_name, i);
            let channel_ident = format_ident!("{}", channel_name_str);

            match_arms.extend(quote! {
                #current_channel_id => {
                    crate::interrupt::typelevel::#channel_ident::set_priority(priority);
                    crate::interrupt::typelevel::#channel_ident::enable();
                }
            });
            current_channel_id += 1;
        }
    }

    // DMAC2 channels (0x80+) - LCPU side, no HCPU interrupt handling
    if let Some(lcpu) = &dma.lcpu {
        for (_, controller) in &lcpu.controllers {
            for i in 1..=controller.channel_total {
                let channel_id = 0x80u8 + (i - 1);
                match_arms.extend(quote! {
                    #channel_id => {
                        // DMAC2 is LCPU-side, no interrupt setup from HCPU
                    }
                });
            }
        }
    }

    implementations.extend(quote! {
        /// Enables the interrupt for a given DMA channel and sets its priority.
        pub unsafe fn enable_dma_channel_interrupt_priority(id: u8, priority: crate::interrupt::Priority) {
            use crate::_generated::interrupt::typelevel::Interrupt;
            match id {
                #match_arms
                _ => panic!("Invalid DMA channel ID"),
            }
        }
    });

    implementations
}

/// Converts a string like "foo_bar" or "foo" to "FooBar" or "Foo".
fn to_pascal_case(s: &str) -> String {
    s.split('_')
        .filter(|part| !part.is_empty())
        .map(|part| {
            let mut chars = part.chars();
            match chars.next() {
                Some(first) => first.to_ascii_uppercase().to_string() + chars.as_str(),
                None => String::new(),
            }
        })
        .collect::<String>()
}

enum GetOneError {
    None,
    Multiple,
}

trait IteratorExt: Iterator {
    fn get_one(self) -> Result<Self::Item, GetOneError>;
}

impl<T: Iterator> IteratorExt for T {
    fn get_one(mut self) -> Result<Self::Item, GetOneError> {
        match self.next() {
            None => Err(GetOneError::None),
            Some(res) => match self.next() {
                Some(_) => Err(GetOneError::Multiple),
                None => Ok(res),
            },
        }
    }
}

/// rustfmt a given path.
/// Failures are logged to stderr and ignored.
fn rustfmt(path: impl AsRef<Path>) {
    let path = path.as_ref();
    match Command::new("rustfmt").args([path]).output() {
        Err(e) => {
            eprintln!("failed to exec rustfmt {:?}: {:?}", path, e);
        }
        Ok(out) => {
            if !out.status.success() {
                eprintln!("rustfmt {:?} failed:", path);
                eprintln!("=== STDOUT:");
                std::io::stderr().write_all(&out.stdout).unwrap();
                eprintln!("=== STDERR:");
                std::io::stderr().write_all(&out.stderr).unwrap();
            }
        }
    }
}
