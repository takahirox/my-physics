use my_physics::validation::{SCENARIOS, ScenarioReport, run_catalog, run_scenario, scenario};
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

fn usage() -> &'static str {
    "usage: maneuver-validation [--scenario NAME|all] [--summary] [--artifacts DIR] [--list]"
}

fn main() {
    if let Err(message) = execute() {
        eprintln!("{message}");
        std::process::exit(2);
    }
}

fn execute() -> Result<(), String> {
    let mut selected = "all".to_owned();
    let mut summary_only = false;
    let mut artifact_dir = None;
    let mut args = std::env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--scenario" => selected = args.next().ok_or_else(|| usage().to_owned())?,
            "--summary" => summary_only = true,
            "--artifacts" => artifact_dir = Some(PathBuf::from(args.next().ok_or_else(|| usage().to_owned())?)),
            "--list" => {
                for definition in SCENARIOS {
                    println!("{}\t{}", definition.name, definition.description);
                }
                return Ok(());
            }
            "--help" | "-h" => {
                println!("{}", usage());
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {argument}\n{}", usage())),
        }
    }
    let reports = if selected == "all" {
        run_catalog()
    } else {
        vec![run_scenario(scenario(&selected).ok_or_else(|| format!("unknown scenario: {selected}"))?)]
    };
    if let Some(directory) = artifact_dir {
        write_artifacts(&directory, &reports)?;
    }
    println!("{}", catalog_json(&reports, summary_only));
    if reports.iter().any(|report| !report.passed()) {
        return Err("one or more maneuver bounds failed".to_owned());
    }
    Ok(())
}

fn catalog_json(reports: &[ScenarioReport], summary_only: bool) -> String {
    let mut output = String::from("{\"schema_version\":1,\"reports\":[");
    for (index, report) in reports.iter().enumerate() {
        if index != 0 {
            output.push(',');
        }
        if summary_only {
            output.push_str(&report.summary_json());
        } else {
            let summary = report.summary_json();
            let timeseries = report.timeseries_json();
            write!(output, "{{\"summary\":{summary},\"timeseries\":{timeseries}}}").unwrap();
        }
    }
    output.push_str("]}");
    output
}

fn write_artifacts(directory: &Path, reports: &[ScenarioReport]) -> Result<(), String> {
    fs::create_dir_all(directory).map_err(|error| format!("create {}: {error}", directory.display()))?;
    for report in reports {
        let base = directory.join(report.definition.name);
        for (suffix, contents) in [
            ("summary.json", report.summary_json()),
            ("summary.csv", report.summary_csv()),
            ("timeseries.json", report.timeseries_json()),
            ("timeseries.csv", report.timeseries_csv()),
        ] {
            let path = base.with_extension(suffix);
            fs::write(&path, contents).map_err(|error| format!("write {}: {error}", path.display()))?;
        }
    }
    Ok(())
}
