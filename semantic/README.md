# Task 4 — Semantic Grounding (REUSE) + SHACL Validation

> 此為 demo 版:`ground_execution.py` / `shapes.ttl` 均為
> **完整參考解**;學生發放版(含 STUDENT TODO)在 `hw3_template/semantic/`。

學生實作兩件事,把 FK/IK 的**執行過程**帶進語意空間並驗證它:

| 實作 | 檔案 | 產物 |
|---|---|---|
| ① Grounding(REUSE 標準本體) | `ground_execution.py` 的 2 個 TODO 函式 | `output/data.ttl` |
| ② SHACL(驗證 shapes) | `shapes.ttl` 的 2 個 TODO shapes | `output/validation.ttl` |

SHACL 除了驗自己的 `data.ttl`,也必須驗**助教提供的 24 筆執行紀錄資料集**
(`ta-faulty-execution.ttl`:好壞混合、含門檻邊界值)。驗證結果會逐筆與
**標準答案 `ta-answer-key.json`** 比對——每筆的旗標(`ARM_OUT_OF_RANGE` /
`NO_CONVERGENCE` / `FK_INACCURATE`)必須與答案**完全一致**
(不多報、不漏報)才算寫對。S3 = 8×(有問題案例答對率) + 2×(乾淨案例答對率)。

評分(30 分)由 `ta-shapes-full.ttl`(TA 完整 SHACL:STRUCTURE 結構檢查
+ 問題偵測 + SHACL-SPARQL 關節限位交叉檢查)與 `score_semantic.py` 自動執行:

```bash
PYTHON=~/miniconda3/envs/pdm-hw3/bin/python bash semantic/run_task4.sh --student-id <id>
```

五步:toolchain → Jena(shacl CLI)→ grounding → SHACL 三份驗證報告
→ 評分。設計原則:**SHACL 直接驗證數值**(門檻:0.90 m reach /
0.02 m residual / 0.005 m pose error),宣告式 constraints 取代
手寫驗證程式。
