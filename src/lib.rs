// SPDX-FileCopyrightText: 2024 Gabriel Marcano
//
// SPDX-License-Identifier: BSD-3-Clause

//! This crate contains traits for interfacing with Sparkfun Variable Loader (SVL) found on
//! Sparkfun's Artemis modules. This tool is derived from Sparkfun's
//! [svl.py](https://github.com/sparkfun/Apollo3_Uploader_SVL) implementation, and extended with a
//! few extra commands.
//!
//! A binary companion crate is provided that is compatible with the original SVL, and is able to
//! run faster, read memory, and use a GPIO to determine whether or not to skip the bootloader
//! completely using an [enhanced bootloader
//! version](https://github.com/gemarcano/svl.git).

pub mod svl;
pub use svl::*;
