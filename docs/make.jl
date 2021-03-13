using Documenter
using FlightMechanicsUtils


makedocs(
    sitename = "Flight Mechanics Utils Documentation",
    modules = [FlightMechanicsUtils],
    pages = ["Home" => "index.md", "API" => "api.md"],
    format = Documenter.HTML(prettyurls = get(ENV, "CI", nothing) == "true"),
)

deploydocs(
    repo = "github.com:AlexS12/FlightMechanicsUtils.jl.git",
    push_preview=true,
    devbranch = "main"
)
