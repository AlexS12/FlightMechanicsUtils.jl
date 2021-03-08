using Documenter
using FlightMechanicsUtils


makedocs(
    sitename = "Flight Mechanics Utils Documentation",
    modules = [FlightMechanicsUtils],
    pages = ["Home" => "index.md", "API" => "api.md"],
    format = Documenter.HTML(prettyurls = false),
)
