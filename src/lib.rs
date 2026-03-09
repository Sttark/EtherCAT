pub mod commands;
pub mod config;
pub mod constants;
pub mod ffi;
pub mod ruckig;
pub mod runtime;
pub mod shm_layout;
pub mod status;
pub mod xml;

#[cfg(feature = "python")]
pub mod python;

#[cfg(feature = "python")]
use pyo3::prelude::*;

#[cfg(feature = "python")]
#[pymodule]
fn _ethercat_rs(py: Python<'_>, module: &Bound<'_, PyModule>) -> PyResult<()> {
    python::register_module(py, module)
}
