# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

# Bash completion for axon unified CLI
#
# Note: This completes top-level commands only.
# For subcommand options, use the full binary name (e.g., axon-recorder --<TAB>).

_axon_completion() {
    local cur words cword
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    words=("${COMP_WORDS[@]}")
    cword=$COMP_CWORD

    local commands="recorder config register refresh transfer panel version help"

    # Complete top-level commands only
    if [[ ${cword} -eq 1 ]]; then
        COMPREPLY=( $(compgen -W "${commands}" -- "${cur}") )
        return 0
    fi

    # No completion for subcommand arguments
    # Use the full binary name for full completion (e.g., axon-recorder<TAB>)
    return 0
}

complete -F _axon_completion axon
