# Copyright (c) 2024 Bjarki Arge Andreasen
# Copyright (c) 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

import pickle
import os
import sys
import argparse

# This is needed to load edt.pickle files
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "dts", "python-devicetree", "src"))
from devicetree import edtlib  # pylint: disable=unused-import

class INTC():
    def __init__(self, intc_node):
        self.ord = intc_node.dep_ordinal
        self.path = intc_node.path
        self.interrupt_lines = intc_node.props['interrupt-lines'].val
        assert self.interrupt_lines > 0, 'interrupt-lines must be a positive value'

    def __lt__(self, other):
        return self.ord < other.ord

    def __eq__(self, other):
        return self.ord == other.ord

    def __hash__(self):
        return hash(self.ord)

class INTL():
    def __init__(self, interrupt):
        self.intc_ord = interrupt.controller.dep_ordinal
        self.intc_path = interrupt.controller.path
        self.intln = interrupt.data['irq']

    def __lt__(self, other):
        return (self.intc_ord, self.intln) < (other.intc_ord, other.intln)

    def __eq__(self, other):
        return self.intc_ord == other.intc_ord and self.intln == other.intln

    def __hash__(self):
        return hash((self.intc_ord, self.intln))

class IRQ():
    def __init__(self, interrupt):
        self.intc_ord = interrupt.controller.dep_ordinal
        self.intc_path = interrupt.controller.path
        self.intln = interrupt.data['irq']
        self.intd_ord = interrupt.node.dep_ordinal
        self.intd_path = interrupt.node.path

    def __lt__(self, other):
        return (self.intc_ord, self.intln, self.intd_ord) < (other.intc_ord, other.intln, other.intd_ord)

    def __eq__(self, other):
        return self.intc_ord == other.intc_ord and self.intln == other.intln and self.intd_ord == other.intd_ord

    def __hash__(self):
        return hash((self.intc_ord, self.intln, self.intd_ord))

class INTBranch():
    def __init__(self, intc: INTC, intln: int):
        self.intc = intc
        self.intln = intln
        self.intl = None
        self.irqs = []
        self.identifier = None

    def intl_on_branch(self, intl: INTL) -> bool:
        return intl.intc_ord == self.intc.ord and self.intln == intl.intln

    def add_intl(self, intl: INTL):
        assert self.intl is None, "only one interrupt line pr branch"
        self.intl = intl

    def irq_on_branch(self, irq: IRQ) -> bool:
        return irq.intc_ord == self.intc.ord and irq.intln == self.intln

    def add_irq(self, irq: INTL):
        assert irq not in self.irqs, "duplicate IRQ"
        self.irqs.append(irq)

    def add_identifier(self, identifier: int):
        assert identifier > -1, 'Idenfifier must be positive'
        self.identifier = identifier

class INTTree():
    def __init__(self, intcs: list[INTC], intls: list[INTL], irqs: list[IRQ]):
        self.tree = []
        self.tree_by_intc = []

        # Add all interrupt lines (branches) to tree
        for intc in intcs:
            self.tree_by_intc.append((intc, [INTBranch(intc, i) for i in range(intc.interrupt_lines)]))
            self.tree += self.tree_by_intc[-1][1]

        # Add implemented interrupt lines to branches
        for intl in intls:
            for branch in self.walk():
                if branch.intl_on_branch(intl):
                    branch.add_intl(intl)
                    break

        # Add interrupt requests to branches
        for irq in irqs:
            for branch in self.walk():
                if branch.irq_on_branch(irq):
                    branch.add_irq(irq)
                    break

        # Store branches of implemented interrupt lines
        self.tree_impl = [branch for branch in self.walk() if branch.intl is not None]

        # Add identifier to branches of implemented interrupt lines
        for identifier, branch in enumerate(self.walk_impl()):
            branch.add_identifier(identifier)

        # Store branches of implemented interrupt lines by interrupt controller
        self.tree_impl_by_intc = []
        for intc, branches in self.walk_by_intc():
            impl_branches = [branch for branch in branches if branch.intl is not None]
            if impl_branches:
                self.tree_impl_by_intc.append((intc, impl_branches))

    def walk(self) -> list[INTBranch]:
        return self.tree

    def walk_by_intc(self) -> list[(INTC, list[INTBranch])]:
        return self.tree_by_intc

    def walk_impl(self) -> list[INTBranch]:
        return self.tree_impl

    def walk_impl_by_intc(self) -> list[(INTC, list[INTBranch])]:
        return self.tree_impl_by_intc

class INTConfig():
    def __init__(self, log_spurious_irq: bool, dynamic_irq: bool):
        self.log_spurious_irq = log_spurious_irq
        self.dynamic_irq = dynamic_irq

def get_interrupts(edt) -> list:
    interrupts = []
    for node in edt.nodes:
        if node.status not in ('okay', 'reserved'):
            continue

        for interrupt in node.interrupts:
            if interrupt in interrupts:
                continue

            interrupts.append(interrupt)

    return interrupts

def get_intc_nodes(edt) -> list:
    return [n for n in edt.nodes if 'interrupt-controller' in n.props]

def get_intcs(intc_nodes) -> list[INTC]:
    return sorted([INTC(intc_node) for intc_node in intc_nodes])

def get_intls(interrupts) -> list[INTL]:
    return sorted({INTL(interrupt) for interrupt in interrupts})

def get_irqs(interrupts) -> list[IRQ]:
    return sorted([IRQ(interrupt) for interrupt in interrupts])

def write_sys_irq_header(f):
    lines = [
        '/*',
        ' * Generated by gen_sys_irq.py',
        ' */',
    ]

    f.write('\n'.join(lines))
    f.write('\n')

def write_sys_dt_irq_intl_map(f, tree: INTTree):
    def format_intl(branch: INTBranch) -> str:
        def format_irq(irq: IRQ) -> str:
            return f' * Interrupt generating device: {irq.intd_path}'

        def format_irqs(irqs: list[IRQ]) -> str:
            return '\n'.join([format_irq(irq) for irq in irqs])

        return '\n'.join([
            '/*',
            f' * Interrupt controller: {branch.intl.intc_path}',
            f' * Interrupt line: {branch.intl.intln}',
            format_irqs(branch.irqs),
            ' */',
            f'#define SYS_DT_IRQN_{branch.intl.intc_ord}_{branch.intl.intln} {branch.identifier}'
        ])

    def format_intls(tree: INTTree) -> str:
        intls = [format_intl(branch) for branch in tree.walk_impl()]
        return '\n\n'.join(intls)

    f.write('\n')
    f.write(format_intls(tree))
    f.write('\n')

def write_sys_dt_irqn_size(f, tree: INTTree):
    lines = [
        '/*',
        ' * Number of implemented interrupt lines',
        ' */',
        f'#define SYS_DT_IRQN_SIZE {len(tree.walk_impl())}',
    ]

    f.write('\n')
    f.write('\n'.join(lines))
    f.write('\n')

def write_sys_irq_handler_externs(f, irqs: list[IRQ]):
    def format_header() -> str:
        return '\n'.join([
            '/*',
            ' * IRQ handler externs',
            ' */',
        ])

    def format_irq(irq: IRQ) -> str:
        return '\n'.join([
            f'extern int __sys_irq_handler_{irq.intc_ord}_{irq.intln}_{irq.intd_ord}(void);',
        ])

    def format_irqs(irqs: list[IRQ]) -> str:
        return '\n'.join([format_irq(irq) for irq in irqs])

    lines = [
        format_header(),
        format_irqs(irqs),
    ]

    f.write('\n')
    f.write('\n'.join(lines))
    f.write('\n')

def write_sys_irq_intl_handlers(f, tree: INTTree, config: INTConfig):
    def format_header(branch: INTBranch) -> str:
        return '\n'.join([
            '/*',
            f' * Interrupt controller: {branch.intc.path}',
            f' * Interrupt line: {branch.intln}',
            ' */',
            f'static inline void __sys_intl_handler_{branch.intc.ord}_{branch.intln}(void)',
            '{',
        ])

    def format_irq_call(irq: IRQ) -> str:
        return '\n'.join([
            '\t/*',
            f'\t * Interrupt generating device: {irq.intd_path}',
            '\t */',
            f'\tif (__sys_irq_handler_{irq.intc_ord}_{irq.intln}_{irq.intd_ord}()) ' + '{',
            '\t\treturn;',
            '\t}',
        ])

    def format_irq_calls(branch: INTBranch) -> list[str]:
        return [format_irq_call(irq) for irq in branch.irqs]

    def format_dynamic_call(branch: INTBranch) -> str:
        return '\n'.join([
            f'\tif (sys_irq_dynamic_handler({branch.identifier})) ' + '{',
            '\t\treturn;',
            '\t}',
        ])

    def format_spurious_log(branch: INTBranch) -> str:
        return f'\tsys_irq_log_spurious_intl({branch.intc.ord}, {branch.intln});'

    def format_spurious_call() -> str:
        return '\tsys_irq_spurious_handler();'

    def format_footer() -> str:
        return '}'

    def format_intl_handlers(tree: INTTree, config: INTConfig) -> list[str]:
        intl_handlers = []

        for branch in tree.walk():
            intl_handler = [format_header(branch)]

            bodyparts = []

            if branch.irqs:
                bodyparts += format_irq_calls(branch)

                if config.dynamic_irq:
                    bodyparts += [format_dynamic_call(branch)]

            if config.log_spurious_irq:
                bodyparts += [format_spurious_log(branch)]

            bodyparts += [format_spurious_call()]
            body = '\n\n'.join(bodyparts)

            intl_handler.append(body)
            intl_handler.append(format_footer())

            intl_handlers.append('\n'.join(intl_handler))

        return intl_handlers

    f.write('\n')
    f.write('\n\n'.join(format_intl_handlers(tree, config)))
    f.write('\n')

def write_sys_dt_irq_foreach_intl_macros(f, tree):
    def format_header(intc: INTC) -> str:
        return '\n'.join([
            '/*',
            f' * Interrupt controller: {intc.path}',
            ' */',
        ])

    def format_intls(intc: INTC, branches: list[INTBranch]) -> str:
        return f'#define SYS_DT_IRQ_INTLS_{intc.ord} {len(branches)}'

    def format_impl_intls(intc: INTC, impl_branches: list[INTBranch]) -> str:
        return f'#define SYS_DT_IRQ_IMPL_INTLS_{intc.ord} {len(impl_branches)}'

    def format_foreach_signature(intc: INTC, name: str, sep: bool, var: bool) -> str:
        return (f'#define SYS_DT_IRQ_FOREACH_{name}_{intc.ord}(id, fn'
                f'{", sep" if sep else ""}'
                f'{", ..." if var else ""}'
                ')')

    def format_foreach_invocation(branch: INTBranch, sep: bool, var: bool) -> str:
        return (f'fn(id, {branch.intln}'
                f'{", __VA_ARGS__" if var else ""}'
                ')'
                f'{" __DEBRACKET sep" if sep else ""}')

    def format_foreach_invocations(branches: list[INTBranch], sep: bool, var: bool) -> list[str]:
        return [format_foreach_invocation(branch, sep, var) for branch in branches]

    def format_foreach_macro(intc: INTC, branches: list[INTBranch], name: str, sep: bool, var: bool) -> str:
        parts = [format_foreach_signature(intc, name, sep, var)]
        parts += format_foreach_invocations(branches, sep, var)
        return ' '.join(parts)

    def format_intc(intc: INTC, branches: list[INTBranch]) -> str:
        impl_branches = [branch for branch in branches if branch.intl is not None]
        return '\n'.join([
            format_header(intc),
            format_intls(intc, branches),
            format_impl_intls(intc, impl_branches),
            format_foreach_macro(intc, branches, 'INTL', False, False),
            format_foreach_macro(intc, branches, 'INTL_SEP', True, False),
            format_foreach_macro(intc, branches, 'INTL_VARGS', False, True),
            format_foreach_macro(intc, branches, 'INTL_SEP_VARGS', True, True),
            format_foreach_macro(intc, impl_branches, 'IMPL_INTL', False, False),
            format_foreach_macro(intc, impl_branches, 'IMPL_INTL_SEP', True, False),
            format_foreach_macro(intc, impl_branches, 'IMPL_INTL_VARGS', False, True),
            format_foreach_macro(intc, impl_branches, 'IMPL_INTL_SEP_VARGS', True, True),
        ])

    def format_intcs(tree: INTTree) -> str:
        return [format_intc(intc, branches) for intc, branches in tree.walk_by_intc()]

    f.write('\n')
    f.write('\n\n'.join(format_intcs(tree)))
    f.write('\n')

def write_sys_irq_intls(f, tree: INTTree):
    def format_header() -> str:
        return '\n'.join([
            '/*',
            ' * Interrupt lines',
            ' */',
            'static const struct sys_irq_intl __sys_irq_intls[] = {',
        ])

    def format_intl(intl: INTL, irqs: list[IRQ]) -> str:
        def format_irq(irq: IRQ) -> str:
            return f'\t * Interrupt generating device: {irq.intd_path}'

        def format_irqs(irqs: list[IRQ]) -> str:
            return '\n'.join([format_irq(irq) for irq in irqs])

        return '\n'.join([
            '\t/*',
            f'\t * Interrupt controller: {intl.intc_path}',
            f'\t * Interrupt line: {intl.intln}',
            format_irqs(irqs),
            '\t */',
            '\t{',
            f'\t\t.intc = &__device_dts_ord_{intl.intc_ord},',
            f'\t\t.intln = {intl.intln}',
            '\t},',
        ])

    def format_intls(tree: INTTree) -> str:
        intls = []
        for branch in tree.walk():
            if branch.intl is None:
                continue

            intls.append(format_intl(branch.intl, branch.irqs))

        return '\n\n'.join(intls)

    def format_footer() -> str:
        return '};'

    lines = [
        format_header(),
        format_intls(tree),
        format_footer(),
    ]

    f.write('\n')
    f.write('\n'.join(lines))
    f.write('\n')

def write_sys_irq_handler_defaults(f, irqs: list[IRQ]):
    def format_header() -> str:
        return '\n'.join([
            '#include <zephyr/sys/irq_handler.h>',
            '#include <zephyr/toolchain.h>',
        ])

    def format_irq(irq: IRQ) -> str:
        return '\n'.join([
            '/*',
            f' * Interrupt controller: {irq.intc_path}',
            f' * Interrupt line: {irq.intln}',
            f' * Interrupt generating device: {irq.intd_path}',
            ' */',
            f'__weak int __sys_irq_handler_{irq.intc_ord}_{irq.intln}_{irq.intd_ord}(void)',
            '{',
            '\treturn 0;',
            '}',
        ])

    def format_irqs(irqs: list[IRQ]) -> str:
        return '\n\n'.join([format_irq(irq) for irq in irqs])

    parts = [
        format_header(),
        format_irqs(irqs),
    ]

    f.write('\n')
    f.write('\n\n'.join(parts))
    f.write('\n')

def parse_args():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--edt-pickle", required=True, help="Path to EDT pickle")
    parser.add_argument("--irq-h", required=True, help="Generated sys irq header output path")
    parser.add_argument("--irq-internal-h", required=True, help="Generated sys irq internal header output path")
    parser.add_argument("--irq-handler-h", required=True, help="Generated sys irq handler header output path")
    parser.add_argument("--irq-handler-c", required=True, help="Generated sys irq handler source output path")
    parser.add_argument("--log-spurious-irq", action='store_true', help="Enable logging of spurious IRQs")
    parser.add_argument("--dynamic-irq", action='store_true', help="Enable dynamic IRQs")
    return parser.parse_args()

def main():
    args = parse_args()

    with open(args.edt_pickle, 'rb') as f:
        edt = pickle.load(f)

    interrupts = get_interrupts(edt)
    intc_nodes = get_intc_nodes(edt)

    intcs = get_intcs(intc_nodes)
    intls = get_intls(interrupts)
    irqs = get_irqs(interrupts)
    tree = INTTree(intcs, intls, irqs)

    config = INTConfig(args.log_spurious_irq, args.dynamic_irq)

    with open(args.irq_h, 'w') as f:
        write_sys_irq_header(f)
        write_sys_dt_irq_intl_map(f, tree)
        write_sys_dt_irqn_size(f, tree)

    with open(args.irq_internal_h, 'w') as f:
        write_sys_irq_header(f)
        write_sys_irq_intls(f, tree)

    with open(args.irq_handler_h, 'w') as f:
        write_sys_irq_header(f)
        write_sys_irq_handler_externs(f, irqs)
        write_sys_irq_intl_handlers(f, tree, config)
        write_sys_dt_irq_foreach_intl_macros(f, tree)

    with open(args.irq_handler_c, 'w') as f:
        write_sys_irq_header(f)
        write_sys_irq_handler_defaults(f, irqs)

if __name__ == "__main__":
    main()
