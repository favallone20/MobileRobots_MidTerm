# Mobile Robots Exam
## Contributing to the development

### How to commit

Commits should be **small** and related to a specific set of changes that are **semantically related to each other**. Although unofficial branches allow for any committing style, short commits are beneficial to keep the repo clean and tidy. If you need to go back to a previous commit or making a-posteriori analyses of your code, finer granularity helps.

In case you need to make a big code refactoring, always remember that you can proceed by committing small incremental work that is still semantically self-contained.

Also, **think before committing!** You should design your commit before typing your `git commit` command, or even before modifying the code. This helps you focusing on the function you are going to implement and better organize your work.

In case you did not think enough, and you made an unfortunate mistake, please read [this guide](https://sethrobertson.github.io/GitFixUm/fixup.html) before trying to solve the problem yourself and possibly stacking additional (unsolvable) mistakes.

### Commit messages

Configure git for using the .gitmessage as commit template issuing the following command:

```bash
git config commit.template .gitmessage
```

this command configures git to use this template only for this project, if you like to configure git to use it for all project you should add the global flag as follows:

```bash
git config --global commit.template ~/.gitmessage
```

When writing commit messages, please use the following conventions

* ADD adding new feature
* FIX a bug
* DOC documentation only
* REF refactoring that doesn't include any changes in features
* FMT formatting only (spacing...)
* MAK repository related changes (e.g., changes in the ignore list)
* TST related to test code only

Use bullet lists for commits including more than one change. **See the latest commit messages for an example before making your first commit!**